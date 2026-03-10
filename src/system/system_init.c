#include "FreeRTOS.h"
#include "task.h"
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/exti.h>

#include "system_init.h"
#include "driver/uart.h"

void system_init(void) {
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
	rcc_periph_clock_enable(RCC_GPIOB);	// I2C
	rcc_periph_clock_enable(RCC_GPIOC);	// LED
	rcc_periph_clock_enable(RCC_AFIO);	// EXTI
	rcc_periph_clock_enable(RCC_I2C1);	// I2C
	uart_init();

	gpio_set_mode(GPIOB,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		GPIO6|GPIO7);			// I2C
	gpio_set(GPIOB,GPIO6|GPIO7);		// Idle high

	gpio_set_mode(GPIOC,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO13);			// LED on PC13
	// gpio_set(GPIOC,GPIO13);			// PC13 LED dark
			     
	// AFIO_MAPR_I2C1_REMAP=0, PB6+PB7
	gpio_primary_remap(0,0); 

	gpio_set_mode(GPIOC,			// PCF8574 /INT
		GPIO_MODE_INPUT,		// Input
		GPIO_CNF_INPUT_FLOAT,
		GPIO14);			// on PC14

	exti_select_source(EXTI14,GPIOC);
	exti_set_trigger(EXTI14,EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI14);
	nvic_enable_irq(NVIC_EXTI15_10_IRQ);	// PC14 <- /INT


}