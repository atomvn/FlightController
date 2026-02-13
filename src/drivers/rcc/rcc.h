#ifndef RCC_H
#define RCC_H
#include "../core/memorymap_f1.h"
#include "../core/common.h"

#define RCC_CR					MMIO32(RCC_BASE + 0x00)
#define RCC_CFGR				MMIO32(RCC_BASE + 0x04)
#define RCC_AHBENR				MMIO32(RCC_BASE + 0x14)
#define RCC_APB2ENR				MMIO32(RCC_BASE + 0x18)
#define RCC_APB1ENR				MMIO32(RCC_BASE + 0x1c)
#define RCC_CSR					MMIO32(RCC_BASE + 0x24)

#define FLASH_ACR			    MMIO32(FLASH_MEM_INTERFACE_BASE + 0x00)

#define RCC_AFIO_EN             (1 << 0)
#define RCC_GPIOA_EN            (1 << 2)
#define RCC_GPIOB_EN            (1 << 3)
#define RCC_UART1_EN            (1 << 14)
#define RCC_I2C1_EN             (1 << 21)

void clock_setup_hsi(void);
void clock_setup_hse(void);
void clock_setup_pll_72mhz(void);
void delay_ms(uint32_t time);

#endif