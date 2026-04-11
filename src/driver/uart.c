/**
 * @file uart.c
 * @brief STM32F1 UART Driver Implementation
 *
 * @details
 *
 * @author Hao Nguyen
 * @version 1.0 
 * @date 2026
 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/usart.h>

#include "uart.h"

/* ======================== Configuration ======================== */

#define UART_INT_MAX_DIGITS   20            ///< Maximum digits for integer conversion
#define UART_FLOAT_MAX_DECIMALS 9           ///< Maximum decimal places supported
#define UART_FORMAT_BUFFER_SIZE 64          ///< Temporary buffer for formatted values
#define UART_HEX_CHARS "0123456789ABCDEF"   ///< Hex character lookup table

/* ======================== Sanity Checks ======================== */

static inline int32_t uart_check_initialized(void) {
    // Check if UART1 is enabled:
    if (!(RCC_APB2ENR & RCC_APB2ENR_USART1EN)) {
        // UART1 not enabled - silent failure (can't print error!)
        return -1;
    }
    return 0;
}

/* ======================== Character I/O ======================== */

/**
 * @brief Send single character to UART1
 * 
 * Blocks until UART is ready and character is transmitted.
 * 
 * @param c Character to send (0x00-0xFF)
 * @return 0 on success, -1 if UART not initialized
 * 
 * @pre UART1 is initialized
 */
int32_t uart_send_char(char c) {
    if (uart_check_initialized() != 0)
        return -1;
    
    usart_send_blocking(USART1, (uint8_t)c);
    return 0;
}

/**
 * @brief Send null-terminated string to UART1
 * 
 * Sends each character in sequence. Stops at null terminator.
 * Safely handles NULL pointer input.
 * 
 * @param str Pointer to string (NULL-safe)
 * @return 0 on success, -1 if UART not initialized or str is NULL
 * 
 * @example
 * uart_send_string("Hello, World!\n");
 */
int32_t uart_send_string(const char *str) {
    if (uart_check_initialized() != 0)
        return -1;
    
    // Null-safe: handle NULL pointer gracefully:
    if (!str) {
        // Print warning without recursing:
        const char msg[] = "[NULL_STRING]";
        for (const char *p = msg; *p; p++) {
            usart_send_blocking(USART1, *p);
        }
        return -1;
    }
    
    // Send each character until null terminator:
    while (*str) {
        usart_send_blocking(USART1, (uint8_t)*str++);
    }
    
    return 0;
}

/* ======================== Numeric Output ======================== */

/**
 * @brief Send unsigned 32-bit integer as decimal string
 * 
 * Converts uint32_t to ASCII decimal representation.
 * Special case: value 0 prints as "0".
 * 
 * @param value Unsigned integer to send (0-4294967295)
 * @return 0 on success
 * 
 * @example
 * uart_send_uint32(12345);  // Sends "12345"
 */
int32_t uart_send_uint32(uint32_t value) {
    if (uart_check_initialized() != 0)
        return -1;
    
    // Special case: zero:
    if (value == 0) {
        uart_send_char('0');
        return 0;
    }
    
    // Convert to decimal string in buffer (reversed):
    char buf[UART_INT_MAX_DIGITS];
    int i = 0;
    
    while (value > 0 && i < UART_INT_MAX_DIGITS) {
        buf[i++] = '0' + (value % 10);
        value /= 10;
    }
    
    // Check for overflow:
    if (i >= UART_INT_MAX_DIGITS) {
        uart_send_string("[NUM_OVERFLOW]");
        return -1;
    }
    
    // Send digits in correct order (reverse of buffer):
    while (i--) {
        uart_send_char(buf[i]);
    }
    
    return 0;
}

/**
 * @brief Send signed 32-bit integer as decimal string
 * 
 * Handles negative numbers correctly (prints "-" prefix).
 * 
 * @param value Signed integer to send
 * @return 0 on success
 * 
 * @example
 * uart_send_int32(-42);  // Sends "-42"
 */
int32_t uart_send_int32(int32_t value) {
    if (uart_check_initialized() != 0)
        return -1;
    
    // Handle negative:
    if (value < 0) {
        uart_send_char('-');
        value = -value;
    }
    
    return uart_send_uint32((uint32_t)value);
}

/**
 * @brief Send floating-point number with decimal places
 * 
 * Sends float as "integer.fraction" format.
 * Precision controlled by decimals parameter.
 * 
 * @param value Floating-point number to send
 * @param decimals Number of decimal places (0-6 recommended)
 * @return 0 on success, -1 if decimals > 9
 * 
 * @example
 * uart_send_float_n(3.14159, 2);  // Sends "3.14"
 * uart_send_float_n(-42.5, 1);    // Sends "-42.5"
 */
int32_t uart_send_float_n(float value, uint8_t decimals) {
    if (uart_check_initialized() != 0)
        return -1;
    
    // Validate decimals:
    if (decimals > UART_FLOAT_MAX_DECIMALS) {
        uart_send_string("[FLOAT_PRECISION_EXCEEDED]");
        return -1;
    }
    
    // Handle negative:
    if (value < 0.0f) {
        uart_send_char('-');
        value = -value;
    }
    
    // Split into integer and fractional parts:
    uint32_t integer = (uint32_t)value;
    
    // Calculate fraction (with rounding):
    float frac = value - (float)integer;
    uint32_t multiplier = 1;
    for (int i = 0; i < decimals; i++) {
        multiplier *= 10;
    }
    
    // Round fractional part:
    uint32_t fraction = (uint32_t)(frac * multiplier + 0.5f);
    
    // Handle case where rounding increases fraction >= multiplier:
    if (fraction >= multiplier) {
        fraction = 0;
        integer++;
    }
    
    // Send integer part:
    uart_send_uint32(integer);
    
    // Send decimal point:
    uart_send_char('.');
    
    // Send fractional part with leading zeros:
    for (uint32_t temp = multiplier / 10; temp > 0 && fraction < temp; temp /= 10) {
        uart_send_char('0');
    }
    
    uart_send_uint32(fraction);
    
    return 0;
}

/**
 * @brief Send floating-point number (legacy, 3 decimal places)
 * 
 * Calls uart_send_float_n() with decimals=3.
 * Kept for backward compatibility.
 * 
 * @param value Floating-point number
 * @return 0 on success
 * 
 * @note Deprecated: Use uart_send_float_n() for custom precision
 */
int32_t uart_send_float(float value) {
    return uart_send_float_n(value, 3);
}

/**
 * @brief Send hexadecimal number (8 digits with 0x prefix)
 * 
 * Prints value as "0xHHHHHHHH" format.
 * Always prints 8 hex digits (zero-padded).
 * 
 * @param value Unsigned integer to print in hex
 * @return 0 on success
 * 
 * @example
 * uart_send_hex(0xDEADBEEF);  // Sends "0xDEADBEEF"
 * uart_send_hex(0x42);        // Sends "0x00000042"
 */
int32_t uart_send_hex(uint32_t value) {
    return uart_send_hex_n(value, 8);
}

/**
 * @brief Send hexadecimal number with variable digit count
 * 
 * Prints value as "0xHH...HH" format with specified number of hex digits.
 * Clamped to 8 digits maximum (32-bit value).
 * 
 * @param value Unsigned integer to print in hex
 * @param digits Number of hex digits to display (1-8)
 * @return 0 on success
 * 
 * @example
 * uart_send_hex_n(0xDEADBEEF, 4);  // Sends "0xBEEF"
 * uart_send_hex_n(0x42, 2);        // Sends "0x42"
 */
int32_t uart_send_hex_n(uint32_t value, uint8_t digits) {
    if (uart_check_initialized() != 0)
        return -1;
    
    // Clamp digits to 8 maximum:
    if (digits > 8)
        digits = 8;
    if (digits < 1)
        digits = 1;
    
    uart_send_string("0x");
    
    // Print hex digits from most significant:
    for (int i = (digits * 4) - 4; i >= 0; i -= 4) {
        uint8_t nibble = (value >> i) & 0xF;
        uart_send_char(UART_HEX_CHARS[nibble]);
    }
    
    return 0;
}

/* ======================== Formatted Output ======================== */

/**
 * @brief Printf-style formatted output to UART1
 * 
 * Supports basic format specifiers:
 * - %d: signed 32-bit integer
 * - %u: unsigned 32-bit integer
 * - %x: hexadecimal (8 digits)
 * - %02d: integer with zero padding to 2 digits
 * - %f: floating-point (default 3 decimals)
 * - %s: null-terminated string
 * - %c: single character
 * - %p: pointer (prints as 0xHHHHHHHH)
 * - %%: literal percent sign
 * 
 * @param fmt Format string (NULL-safe)
 * @param ... Variable arguments
 * @return 0 on success, -1 if fmt is NULL or error
 * 
 * @note UART must be initialized before calling
 * 
 * @example
 * uart_printf("[INFO] Temperature: %f°C, Status: 0x%02x\n", temp, status);
 * uart_printf("Servo position: %u µs, Motor throttle: %u%%\n", pulse, percent);
 */
static const char* uart_parse_width(const char *fmt, uint8_t *width_out, bool *zero_pad_out) {
    *width_out = 0;
    *zero_pad_out = false;
    
    // Check for zero padding:
    if (*fmt == '0') {
        *zero_pad_out = true;
        fmt++;
    }
    
    // Parse width digits:
    while (*fmt >= '0' && *fmt <= '9') {
        *width_out = (*width_out * 10) + (*fmt - '0');
        fmt++;
    }
    
    return fmt;
}

/**
 * @brief Internal helper: Print integer with zero padding
 */
static void uart_print_padded_int(int32_t value, uint8_t width) {
    // Convert to string first:
    char buf[UART_INT_MAX_DIGITS];
    int len = 0;
    
    bool negative = (value < 0);
    if (negative) {
        uart_send_char('-');
        value = -value;
        width--;  // Account for minus sign
    }
    
    uint32_t temp = value;
    do {
        buf[len++] = '0' + (temp % 10);
        temp /= 10;
    } while (temp > 0);
    
    // Print leading zeros:
    while (len < width) {
        uart_send_char('0');
        len++;
    }
    
    // Print digits in reverse:
    while (len--) {
        uart_send_char(buf[len]);
    }
}

int32_t uart_printf(const char *fmt, ...) {
    if (uart_check_initialized() != 0)
        return -1;
    
    // Null-safe: handle NULL format string:
    if (!fmt) {
        const char msg[] = "[NULL_FORMAT]\n";
        for (const char *p = msg; *p; p++) {
            usart_send_blocking(USART1, *p);
        }
        return -1;
    }
    
    va_list args;
    va_start(args, fmt);
    
    while (*fmt) {
        // Print non-format characters directly:
        if (*fmt != '%') {
            uart_send_char(*fmt++);
            continue;
        }
        
        fmt++;  // Skip '%'
        
        // Parse width and flags:
        uint8_t width = 0;
        bool zero_pad = false;
        fmt = uart_parse_width(fmt, &width, &zero_pad);
        
        // Handle format specifier:
        switch (*fmt) {
        case '%':
            // Literal percent:
            uart_send_char('%');
            break;
        
        case 'd': {
            // Signed integer:
            int32_t value = va_arg(args, int32_t);
            if (width > 0 && zero_pad) {
                uart_print_padded_int(value, width);
            } else {
                uart_send_int32(value);
            }
            break;
        }
        
        case 'u': {
            // Unsigned integer:
            uint32_t value = va_arg(args, uint32_t);
            uart_send_uint32(value);
            break;
        }
        
        case 'x': {
            // Hexadecimal:
            uint32_t value = va_arg(args, uint32_t);
            if (width > 0 && width <= 8) {
                uart_send_hex_n(value, width);
            } else {
                uart_send_hex(value);
            }
            break;
        }
        
        case 'f': {
            // Float (default 3 decimals, or use width as decimals):
            double value = va_arg(args, double);
            uint8_t decimals = (width > 0 && width <= UART_FLOAT_MAX_DECIMALS) ? width : 3;
            uart_send_float_n((float)value, decimals);
            break;
        }
        
        case 's': {
            // String:
            const char *str = va_arg(args, const char *);
            if (str) {
                uart_send_string(str);
            } else {
                uart_send_string("[NULL_STR]");
            }
            break;
        }
        
        case 'c': {
            // Character:
            int ch = va_arg(args, int);
            uart_send_char((char)ch);
            break;
        }
        
        case 'p': {
            // Pointer:
            uint32_t ptr = va_arg(args, uint32_t);
            uart_send_hex(ptr);
            break;
        }
        
        default:
            // Unknown format specifier:
            uart_send_char('%');
            uart_send_char(*fmt);
        }
        
        fmt++;
    }
    
    va_end(args);
    
    return 0;
}
