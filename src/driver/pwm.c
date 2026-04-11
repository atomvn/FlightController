/**
 * @file pwm.c
 * @brief STM32F1 PWM Driver Implementation
 *
 * @details
 *
 * @author Hao Nguyen
 * @version 1.0 
 * @date 2026
 */
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/timer.h>

#include "driver/uart.h"
#include "pwm.h"

/* ======================== Configuration Presets ======================== */

const pwm_config_t PWM_CONFIG_SERVO = {
    .min_pulse_us = PWM_MIN_US,
    .max_pulse_us = PWM_MAX_US,
    .neutral_pulse_us = PWM_NEUTRAL_US,
    .frequency_hz = 50
};

const pwm_config_t PWM_CONFIG_MOTOR = {
    .min_pulse_us = PWM_MOTOR_MIN_US,
    .max_pulse_us = PWM_MOTOR_MAX_US,
    .neutral_pulse_us = 1000,  // Motor starts at min throttle
    .frequency_hz = 50
};

/* ======================== Module State ======================== */

static struct {
    bool motor_initialized;
    bool servos_initialized;
    uint16_t current_throttle_us;
    uint16_t servo_positions[2];
    pwm_config_t motor_config;
    pwm_config_t servo_config;
} pwm_state = {
    .motor_initialized = false,
    .servos_initialized = false,
    .current_throttle_us = PWM_MIN_US,
    .servo_positions = {PWM_NEUTRAL_US, PWM_NEUTRAL_US},
    .motor_config = PWM_CONFIG_MOTOR,
    .servo_config = PWM_CONFIG_SERVO
};

/* ======================== Helper Functions ======================== */

/**
 * @brief Validate PWM configuration parameters
 * @return 0 if valid, -1 if invalid
 */
static int32_t pwm_config_validate(const pwm_config_t *config) {
    if (!config) return -1;
    
    // Validate ranges:
    if (config->min_pulse_us >= config->max_pulse_us) {
        uart_printf("[ERROR] PWM: min_pulse_us >= max_pulse_us (%u >= %u)\n",
                   config->min_pulse_us, config->max_pulse_us);
        return -1;
    }
    
    if (config->neutral_pulse_us < config->min_pulse_us ||
        config->neutral_pulse_us > config->max_pulse_us) {
        uart_printf("[ERROR] PWM: neutral_pulse_us out of range (%u, valid: %u-%u)\n",
                   config->neutral_pulse_us, config->min_pulse_us, config->max_pulse_us);
        return -1;
    }
    
    if (config->frequency_hz < 10 || config->frequency_hz > 1000) {
        uart_printf("[ERROR] PWM: frequency out of range (%u Hz, valid: 10-1000)\n",
                   config->frequency_hz);
        return -1;
    }
    
    return 0;
}

/**
 * @brief Clamp pulse width to valid range
 */
static uint16_t pwm_clamp_pulse(uint16_t pulse_us, const pwm_config_t *config) {
    if (pulse_us < config->min_pulse_us)
        return config->min_pulse_us;
    if (pulse_us > config->max_pulse_us)
        return config->max_pulse_us;
    return pulse_us;
}

/* ======================== Motor PWM Initialization ======================== */

int32_t pwm_init_motor(const pwm_config_t *config) {
    if (pwm_state.motor_initialized) {
        uart_printf("[WARN] Motor PWM already initialized\n");
        return 0;  // Already initialized, not an error
    }
    
    // Use default config if not provided:
    if (config) {
        if (pwm_config_validate(config) != 0) {
            return -1;
        }
        pwm_state.motor_config = *config;
    }
    
    uart_printf("[LOG] Initializing motor PWM: %u Hz, %u-%u µs\n",
               pwm_state.motor_config.frequency_hz,
               pwm_state.motor_config.min_pulse_us,
               pwm_state.motor_config.max_pulse_us);
    
    // Enable clocks:
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_GPIOA);
    
    // Configure GPIO: PA0 as Timer2 output (alternate function, push-pull):
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0);
    
    // Reset timer:
    rcc_periph_reset_pulse(RST_TIM2);
    
    // Configure timer:
    timer_set_prescaler(TIM2, PWM_PRESCALER);
    
    // Period based on frequency:
    // period_ticks = 1MHz / frequency_hz
    uint32_t period_ticks = PWM_TIMER_FREQUENCY_HZ / pwm_state.motor_config.frequency_hz;
    timer_set_period(TIM2, period_ticks - 1);
    
    // Configure OC1 (PA0):
    timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM2, TIM_OC1);
    
    // Set to minimum throttle:
    pwm_state.current_throttle_us = pwm_state.motor_config.min_pulse_us;
    timer_set_oc_value(TIM2, TIM_OC1, pwm_state.current_throttle_us);
    
    // Enable counter:
    timer_enable_counter(TIM2);
    
    pwm_state.motor_initialized = true;
    
    uart_printf("[OK] Motor PWM initialized on TIM2_CH1 (PA0)\n");
    
    return 0;
}

/* ======================== Servo PWM Initialization ======================== */

int32_t pwm_init_servos(const pwm_config_t *config) {
    if (pwm_state.servos_initialized) {
        uart_printf("[WARN] Servo PWM already initialized\n");
        return 0;
    }
    
    // Use default config if not provided:
    if (config) {
        if (pwm_config_validate(config) != 0) {
            return -1;
        }
        pwm_state.servo_config = *config;
    }
    
    uart_printf("[LOG] Initializing servo PWM: %u Hz, %u-%u µs\n",
               pwm_state.servo_config.frequency_hz,
               pwm_state.servo_config.min_pulse_us,
               pwm_state.servo_config.max_pulse_us);
    
    // Enable clocks:
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_GPIOA);
    
    // Configure GPIO: PA6, PA7 as Timer3 outputs:
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO6 | GPIO7);
    
    // Reset timer:
    rcc_periph_reset_pulse(RST_TIM3);
    
    // Configure timer:
    timer_set_prescaler(TIM3, PWM_PRESCALER);
    
    // Period:
    uint32_t period_ticks = PWM_TIMER_FREQUENCY_HZ / pwm_state.servo_config.frequency_hz;
    timer_set_period(TIM3, period_ticks - 1);
    
    // Configure OC1 (PA6 - Servo 0):
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM3, TIM_OC1);
    timer_set_oc_value(TIM3, TIM_OC1, pwm_state.servo_config.neutral_pulse_us);
    pwm_state.servo_positions[0] = pwm_state.servo_config.neutral_pulse_us;
    
    // Configure OC2 (PA7 - Servo 1):
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM3, TIM_OC2);
    timer_set_oc_value(TIM3, TIM_OC2, pwm_state.servo_config.neutral_pulse_us);
    pwm_state.servo_positions[1] = pwm_state.servo_config.neutral_pulse_us;
    
    // Enable counter:
    timer_enable_counter(TIM3);
    
    pwm_state.servos_initialized = true;
    
    uart_printf("[OK] Servo PWM initialized on TIM3_CH1,CH2 (PA6,PA7)\n");
    
    return 0;
}

/* ======================== Motor Control ======================== */

int32_t pwm_set_throttle(uint16_t pulse_us) {
    if (!pwm_state.motor_initialized) {
        uart_printf("[ERROR] Motor PWM not initialized\n");
        return -1;
    }
    
    // Clamp to valid range:
    uint16_t clamped = pwm_clamp_pulse(pulse_us, &pwm_state.motor_config);
    
    if (clamped != pulse_us) {
        uart_printf("[WARN] Throttle %u µs clamped to %u µs (range: %u-%u)\n",
                   pulse_us, clamped,
                   pwm_state.motor_config.min_pulse_us,
                   pwm_state.motor_config.max_pulse_us);
    }
    
    // Update hardware:
    timer_set_oc_value(TIM2, TIM_OC1, clamped);
    pwm_state.current_throttle_us = clamped;
    
    return 0;
}

uint16_t pwm_get_throttle(void) {
    return pwm_state.current_throttle_us;
}

int32_t pwm_set_motor_min(void) {
    return pwm_set_throttle(pwm_state.motor_config.min_pulse_us);
}

/* ======================== Servo Control ======================== */

int32_t pwm_set_servo(uint8_t servo_id, uint16_t pulse_us) {
    if (!pwm_state.servos_initialized) {
        uart_printf("[ERROR] Servo PWM not initialized\n");
        return -1;
    }
    
    if (servo_id >= 2) {
        uart_printf("[ERROR] Invalid servo ID %u (valid: 0-1)\n", servo_id);
        return -1;
    }
    
    // Clamp to valid range:
    uint16_t clamped = pwm_clamp_pulse(pulse_us, &pwm_state.servo_config);
    
    if (clamped != pulse_us) {
        uart_printf("[WARN] Servo %u pulse %u µs clamped to %u µs (range: %u-%u)\n",
                   servo_id, pulse_us, clamped,
                   pwm_state.servo_config.min_pulse_us,
                   pwm_state.servo_config.max_pulse_us);
    }
    
    // Update hardware:
    if (servo_id == 0) {
        timer_set_oc_value(TIM3, TIM_OC1, clamped);
    } else {
        timer_set_oc_value(TIM3, TIM_OC2, clamped);
    }
    
    pwm_state.servo_positions[servo_id] = clamped;
    
    return 0;
}

uint16_t pwm_get_servo(uint8_t servo_id) {
    if (servo_id >= 2) {
        return 0;
    }
    return pwm_state.servo_positions[servo_id];
}

int32_t pwm_set_servos_neutral(void) {
    if (!pwm_state.servos_initialized)
        return -1;
    
    pwm_set_servo(0, pwm_state.servo_config.neutral_pulse_us);
    pwm_set_servo(1, pwm_state.servo_config.neutral_pulse_us);
    
    return 0;
}
