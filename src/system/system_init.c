#include "FreeRTOS.h"
#include "task.h"
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/exti.h>
#include <libopencm3/stm32/f1/usart.h>

#include "system_init.h"
#include "driver/uart.h"
#include "driver/pwm.h"
#include "util/sensor_fusion.h"


void system_init(void) {
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
	rcc_periph_clock_enable(RCC_GPIOB);	// I2C
	rcc_periph_clock_enable(RCC_GPIOC);	// LED
	rcc_periph_clock_enable(RCC_AFIO);	// EXTI
	rcc_periph_clock_enable(RCC_I2C1);	// I2C

	/* UART1 init for logging task*/
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);
	gpio_set_mode(GPIOA,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		GPIO_USART1_TX); // UART1 TX on PA9
	usart_set_baudrate(USART1,38400);
	usart_set_databits(USART1,8);
	usart_set_stopbits(USART1,USART_STOPBITS_1);
	usart_set_mode(USART1,USART_MODE_TX);
	usart_set_parity(USART1,USART_PARITY_NONE);
	usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);
	usart_enable(USART1);

	/* UART2 init for reading receiver task*/
	rcc_periph_clock_enable(RCC_USART2);
	gpio_set_mode(GPIOA,
		GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_FLOAT,
		GPIO_USART2_RX);
	usart_set_baudrate(USART2,100000);
	usart_set_databits(USART2,8);
	usart_set_stopbits(USART2,USART_STOPBITS_2);
	usart_set_mode(USART2,USART_MODE_RX);
	usart_set_parity(USART2,USART_PARITY_EVEN);
	usart_enable(USART2);

	// Pin configure for I2C
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

	pwm_init_brushless_motor();
    pwm_init_2_servos();
}