#include <stdio.h>
#include <stdarg.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/usart.h>

#include "uart.h"

void uart_send_char(char c) {
	usart_send_blocking(USART1, c);
}

void uart_send_string(const char *str) {
	while (*str) {
		usart_send_blocking(USART1, *str++);
	}
}

void uart_send_uint32(uint32_t value) {
	char buf[10];
    int i = 0;

    if (value == 0) {
        uart_send_char('0');
        return;
    }

    while (value > 0) {
        buf[i++] = '0' + (value % 10);
        value /= 10;
    }

    while (i--) {
        uart_send_char(buf[i]);
    }
}

void uart_send_int32(int32_t value)
{
    if (value < 0) {
        uart_send_char('-');
        value = -value;
    }

    uart_send_uint32((uint32_t)value);
}

void uart_send_float(float value)
{
    if (value < 0) {
        uart_send_char('-');
        value = -value;
    }

    uint32_t integer = (uint32_t)value;
    uint32_t fraction = (uint32_t)((value - integer) * 1000); // 3 decimal

    uart_send_uint32(integer);
    uart_send_char('.');

    if (fraction < 100) uart_send_char('0');
    if (fraction < 10) uart_send_char('0');

    uart_send_uint32(fraction);
}

void uart_send_hex(uint32_t value)
{
    char hex[] = "0123456789ABCDEF";

    uart_send_string("0x");

    for (int i = 28; i >= 0; i -= 4) {
        uart_send_char(hex[(value >> i) & 0xF]);
    }
}

void uart_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    while (*fmt) {

        if (*fmt != '%') {
            uart_send_char(*fmt++);
            continue;
        }

        fmt++;

        switch (*fmt) {

        case 'd':
            uart_send_int32(va_arg(args, int32_t));
            break;

        case 'u':
            uart_send_uint32(va_arg(args, uint32_t));
            break;

        case 'x':
            uart_send_hex(va_arg(args, uint32_t));
            break;

        case 'f':
            uart_send_float((float)va_arg(args, double));
            break;

        case 's':
            uart_send_string(va_arg(args, char *));
            break;

        case 'c':
            uart_send_char((char)va_arg(args, int));
            break;

        default:
            uart_send_char('%');
            uart_send_char(*fmt);
        }

        fmt++;
    }

    va_end(args);
}

