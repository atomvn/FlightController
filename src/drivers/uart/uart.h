#ifndef UART_H
#define UART_H

#include "../core/memorymap_f1.h"
#include "../core/common.h"
#include "drivers/rcc/rcc.h"
#include "drivers/gpio/gpio.h"

#define USART1				    USART1_BASE
/* Status register (USARTx_SR) */
#define USART_SR(usart_base)	MMIO32((usart_base) + 0x00)
#define USART1_SR			    USART_SR(USART1_BASE)

/* Data register (USARTx_DR) */
#define USART_DR(usart_base)	MMIO32((usart_base) + 0x04)
#define USART1_DR			    USART_DR(USART1_BASE)

/* Baud rate register (USARTx_BRR) */
#define USART_BRR(usart_base)		MMIO32((usart_base) + 0x08)
#define USART1_BRR			USART_BRR(USART1_BASE)

/* Control register 1 (USARTx_CR1) */
#define USART_CR1(usart_base)		MMIO32((usart_base) + 0x0c)
#define USART1_CR1			USART_CR1(USART1_BASE)

/* Control register 2 (USARTx_CR2) */
#define USART_CR2(usart_base)		MMIO32((usart_base) + 0x10)
#define USART1_CR2			USART_CR2(USART1_BASE)

/* Control register 3 (USARTx_CR3) */
#define USART_CR3(usart_base)		MMIO32((usart_base) + 0x14)
#define USART1_CR3			USART_CR3(USART1_BASE)


/* SR bits */
#define USART_SR_TXE  (1 << 7)
#define USART_SR_RXNE (1 << 5)

/* CR1 bits */
#define USART_CR1_UE  (1 << 13)
#define USART_CR1_TE  (1 << 3)
#define USART_CR1_RE  (1 << 2)

void uart1_init(void);
void uart1_send_char(char c);
void uart1_send_string(const char *s);
char uart1_receive_char(void);
void uart1_send_int16(int16_t num);
void uart1_send_float(float value, uint8_t precision);

#endif