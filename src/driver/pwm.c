#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/timer.h>

#include "pwm.h"

void pwm_init_brushless_motor(void) {
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_GPIOA);

    // PA0 = TIM2_CH1
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0);

    rcc_periph_reset_pulse(RST_TIM2);

    // Timer clock = 72MHz
    // Prescaler = 72 → 1MHz
    timer_set_prescaler(TIM2, 72 - 1);

    // Period = 20000 → 20ms
    timer_set_period(TIM2, 20000 - 1);

    // PWM mode 1
    timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM2, TIM_OC1);

    timer_set_oc_value(TIM2, TIM_OC1, 1000); // start min throttle

    timer_enable_counter(TIM2);
}

void pwm_init_2_servos(void) {
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_GPIOA);

    // PA6, PA7 = TIM3_CH1, CH2
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO6 | GPIO7);

    rcc_periph_reset_pulse(RST_TIM3);

    timer_set_prescaler(TIM3, 72 - 1);   // 1MHz
    timer_set_period(TIM3, 20000 - 1);   // 20ms

    // CH1
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM3, TIM_OC1);

    // CH2
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM3, TIM_OC2);

    // Mid position
    timer_set_oc_value(TIM3, TIM_OC1, 1500);
    timer_set_oc_value(TIM3, TIM_OC2, 1500);

    timer_enable_counter(TIM3);
}