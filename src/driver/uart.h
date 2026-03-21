#ifndef UART_H
#define UART_H

void uart_send_char(char c);
void uart_send_string(const char *str);
void uart_send_uint32(uint32_t value);
void uart_send_int32(int32_t value);
void uart_send_float(float value);
void uart_send_hex(uint32_t value);
/* uart_printf allows receiving variable number of args thanks to variadic arguments (...) and stdarg.h lib*/
void uart_printf(const char *format, ...);

#endif