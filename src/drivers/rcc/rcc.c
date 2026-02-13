#include "rcc.h"

// Set up HSI clock 8MHz
void clock_setup_hsi(void) {
        /* 1. Enable HSI */
    RCC_CR |= (1 << 0);                 // HSION

    /* 2. Wait until HSI ready */
    while (!(RCC_CR & (1 << 1)));       // HSIRDY

    /* 3. Flash wait state = 0 (8MHz) */
    FLASH_ACR &= ~(0x7);                // LATENCY = 0

    /* 4. Select HSI as SYSCLK */
    RCC_CFGR &= ~(0x3);                 // SW = 00 (HSI)

    /* 5. Wait SYSCLK switch */
    while (((RCC_CFGR >> 2) & 0x3) != 0x0);  // SWS = HSI
}

// Set up HSE clock 8MHz
void clock_setup_hse(void)
{
    /* 1. Enable HSE */
    RCC_CR |= (1 << 16);                // HSEON

    /* 2. Wait until ready */
    while (!(RCC_CR & (1 << 17)));      // HSERDY

    /* 3. Flash wait state = 0 */
    FLASH_ACR &= ~(0x7);

    /* 4. Select HSE as SYSCLK */
    RCC_CFGR &= ~(0x3);
    RCC_CFGR |=  (0x1);                 // SW = 01 (HSE)

    /* 5. Wait switch */
    while (((RCC_CFGR >> 2) & 0x3) != 0x1);
}

// Set up sys clock using pll 72MHz, source clock HSE 
void clock_setup_pll_72mhz(void) {
    /* 1. Enable HSE */
    RCC_CR |= (1 << 16);
    while (!(RCC_CR & (1 << 17)));

    /* 2. Flash wait state = 2 */
    FLASH_ACR &= ~(0x7);
    FLASH_ACR |=  (2 << 0);

    /* 3. Prescalers */
    RCC_CFGR &= ~(0xF << 4);             // HPRE = 1
    RCC_CFGR &= ~(0x7 << 8);
    RCC_CFGR |=  (0x4 << 8);             // PPRE1 = /2
    RCC_CFGR &= ~(0x7 << 11);            // PPRE2 = /1

    /* 4. PLL config */
    RCC_CFGR |=  (1 << 16);              // PLLSRC = HSE
    RCC_CFGR &= ~(0xF << 18);
    RCC_CFGR |=  (7 << 18);              // PLL ×9

    /* 5. Enable PLL */
    RCC_CR |= (1 << 24);
    while (!(RCC_CR & (1 << 25)));

    /* 6. Switch SYSCLK to PLL */
    RCC_CFGR &= ~(0x3);
    RCC_CFGR |=  (0x2);                  // SW = PLL

    while (((RCC_CFGR >> 2) & 0x3) != 0x2);
}

void delay_ms(uint32_t time) {
    for (int i = 0; i < time; i++) {
        asm("nop");
    }
}