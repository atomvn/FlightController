#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>

/* ======================== Public API ======================== */
/** @brief Send single character to UART1
 * @param c Character to send (0x00-0xFF)
 * @return 0 on success, -1 if UART not initialized
 */
int32_t uart_send_char(char c);

/** @brief Send null-terminated string to UART1
 * @param str Pointer to string (NULL-safe)
 * @return 0 on success, -1 if UART not initialized or str is NULL
 */
int32_t uart_send_string(const char *str);

/** @brief Send unsigned 32-bit integer as decimal string
 * @param value Unsigned integer to send
 * @return 0 on success, -1 if UART not initialized
 */
int32_t uart_send_uint32(uint32_t value);

/** @brief Send signed 32-bit integer as decimal string
 * @param value Signed integer to send
 * @return 0 on success, -1 if UART not initialized
 */
int32_t uart_send_int32(int32_t value);

/** @brief Send floating-point number with decimal places
 * @param value Floating-point number to send
 * @param decimals Number of decimal places (0-9)
 * @return 0 on success, -1 if UART not initialized or decimals > 9
 */
int32_t uart_send_float_n(float value, uint8_t decimals);

/** @brief Send floating-point number with default 3 decimal places
 * @param value Floating-point number to send
 * @return 0 on success, -1 if UART not initialized
 */
int32_t uart_send_float(float value);

/** @brief Send unsigned 32-bit integer as hexadecimal string
 * @param value Unsigned integer to print in hex
 * @return 0 on success, -1 if UART not initialized
 * @example
 * uart_send_hex(0xDEADBEEF);  // Sends "0xDEADBEEF"
 * uart_send_hex(0x42);        // Sends "0x00000042"
 */
int32_t uart_send_hex(uint32_t value);

/** @brief Send unsigned 32-bit integer as hexadecimal string with variable digit count
 * @param value Unsigned integer to print in hex
 * @param digits Number of hex digits to display (1-8)
 * @return 0 on success, -1 if UART not initialized
 * @example
 * uart_send_hex_n(0xDEADBEEF, 4);  // Sends "0xBEEF"
 * uart_send_hex_n(0x42, 2);        // Sends "0x42"
 */
int32_t uart_send_hex_n(uint32_t value, uint8_t digits);

/** @brief Send formatted string to UART1
 * @param fmt Format string (printf-style)
 * @return 0 on success, -1 if UART not initialized
 */
int32_t uart_printf(const char *fmt, ...);

#endif  // UART_H
