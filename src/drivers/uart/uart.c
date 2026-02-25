#include "uart.h"

// Init UART1
void uart1_init(void) {
    /* Enable GPIOA + USART1 clock */
    RCC_APB2ENR |= RCC_GPIOA_EN | RCC_UART1_EN;

    /* PA9 = TX (AF push-pull, 50MHz)
    PA10 = RX (input floating) */
    gpio_mode_setup(GPIOA, 9, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL); // PA9: AF PP, 50MHz
    gpio_mode_setup(GPIOA, 10, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT); //PA10: input floating

    /* Baudrate 9600 @72MHz */
    USART1_BRR = (468 << 4) | 12;

    /* Enable USART, TX, RX */
    USART1_CR1 = USART_CR1_UE |
                 USART_CR1_TE |
                 USART_CR1_RE;
}

// Send a byte / char via uart1
void uart1_send_char(char c) {
    while (!(USART1_SR & USART_SR_TXE));
    USART1_DR = c;
}

// Send a string via uart1
void uart1_send_string(const char *s)
{
    while (*s) {
        uart1_send_char(*s++);
    }
}

// Receive a char via uart1
char uart1_receive_char(void)
{
    while (!(USART1_SR & USART_SR_RXNE));
    return USART1_DR;
}

// Send a int16_t number via uart1
void uart1_send_int16(int16_t value) {

    if (value < 0) {
        uart1_send_char('-');
        value = -value;
    }
    char buff[6];
    int i = 0;

    if (value == 0) {
        uart1_send_char('0');
        return;
    }

    while (value > 0) {
        buff[i++] = (value%10) + '0';
        value /= 10;
    }

    while (i--) {
        uart1_send_char(buff[i]);
    }
}

// Send a float number via uart 1
void uart1_send_float(float value, uint8_t precision)
{
    if (value < 0) {
        uart1_send_char('-');
        value = -value;
    }

    // Integer part
    int16_t int_part = (int16_t)value;
    uart1_send_int16(int_part);

    uart1_send_char('.');

    // Fraction part
    float frac = value - int_part;

    for (uint8_t i = 0; i < precision; i++) {
        frac *= 10;
        int digit = (int)frac;
        uart1_send_char(digit + '0');
        frac -= digit;
    }
}

