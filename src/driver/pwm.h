#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include <stdbool.h>

/* ======================== PWM Configuration ======================== */

/**
 * @brief PWM timing constants
 * 
 * Standard servo/ESC protocol:
 * - Frequency: 50 Hz (20 ms period)
 * - Pulse width: 1000-2000 µs (neutral at 1500 µs)
 * - Timer clock: 72 MHz / 72 = 1 MHz (1 µs per tick)
 */
#define PWM_TIMER_CLOCK_MHZ      72                ///< STM32F1 timer clock
#define PWM_PRESCALER            (PWM_TIMER_CLOCK_MHZ - 1)  ///< Divide to 1 MHz
#define PWM_TIMER_FREQUENCY_HZ   1000000           ///< After prescaler
#define PWM_PERIOD_MS            20                ///< 50 Hz = 20 ms

#define PWM_TICKS_PER_US         1                 ///< At 1 MHz timer clock
#define PWM_PERIOD_TICKS         (PWM_TIMER_FREQUENCY_HZ * PWM_PERIOD_MS / 1000)  ///< 20000

#define PWM_NEUTRAL_US           1500              ///< Servo neutral position
#define PWM_MIN_US               1000              ///< Minimum pulse width
#define PWM_MAX_US               2000              ///< Maximum pulse width
#define PWM_MOTOR_MIN_US         1000              ///< Motor min throttle
#define PWM_MOTOR_MAX_US         2000              ///< Motor max throttle

/* ======================== PWM Configuration Structure ======================== */

/**
 * @brief PWM configuration for flexible initialization
 * 
 * Allows custom pulse width ranges and frequency settings.
 * Use preset configurations for standard servo/ESC operation.
 */
typedef struct {
    uint16_t min_pulse_us;    ///< Minimum pulse width (µs)
    uint16_t max_pulse_us;    ///< Maximum pulse width (µs)
    uint16_t neutral_pulse_us; ///< Neutral/center position (µs)
    uint16_t frequency_hz;    ///< PWM frequency (Hz) - typically 50 for servos
} pwm_config_t;

/**
 * @brief Standard servo configuration (50 Hz, 1000-2000 µs)
 */
extern const pwm_config_t PWM_CONFIG_SERVO;

/**
 * @brief Standard motor/ESC configuration (50 Hz, 1000-2000 µs)
 */
extern const pwm_config_t PWM_CONFIG_MOTOR;

/* ======================== Public API ======================== */

/**
 * @brief Initialize brushless motor ESC PWM on TIM2_CH1
 * 
 * Configures Timer 2 Channel 1 for 50 Hz PWM output on PA0.
 * Initializes to minimum throttle (1000 µs).
 * 
 * @param config PWM configuration (NULL → use PWM_CONFIG_MOTOR)
 * 
 * @return 0 on success, -1 if configuration is invalid
 * 
 * @pre GPIO and Timer2 hardware are available
 * @post TIM2_CH1 outputs PWM on PA0 at configured frequency
 * @post Initial throttle is at min_pulse_us
 * 
 * @example
 * if (pwm_init_motor(NULL) != 0) {
 *     uart_printf("Motor PWM init failed\n");
 * }
 * pwm_set_throttle(1500);  // 50% throttle
 */
int32_t pwm_init_motor(const pwm_config_t *config);

/**
 * @brief Initialize dual servo PWM outputs on TIM3_CH1 and TIM3_CH2
 * 
 * Configures Timer 3 Channels 1 and 2 for servo control on PA6 and PA7.
 * Both channels output 50 Hz PWM initialized to neutral (1500 µs).
 * 
 * @param config PWM configuration (NULL → use PWM_CONFIG_SERVO)
 * 
 * @return 0 on success, -1 if configuration is invalid
 * 
 * @pre GPIO and Timer3 hardware are available
 * @post TIM3_CH1 outputs PWM on PA6, TIM3_CH2 on PA7
 * @post Both servos start at neutral position
 * 
 * @example
 * if (pwm_init_servos(NULL) != 0) {
 *     uart_printf("Servo PWM init failed\n");
 * }
 * pwm_set_servo(0, 1300);  // Servo 0 to position
 * pwm_set_servo(1, 1700);  // Servo 1 to position
 */
int32_t pwm_init_servos(const pwm_config_t *config);

/**
 * @brief Set motor throttle pulse width
 * 
 * Updates motor ESC throttle command on TIM2_CH1.
 * Value is clamped to configured min/max range.
 * 
 * @param pulse_us Pulse width in microseconds (1000-2000 typical)
 * 
 * @return 0 on success, -1 if pulse_us is out of range
 * 
 * @pre pwm_init_motor() has been called
 * @post Motor throttle updated, range enforced
 * 
 * @example
 * // Ramp throttle from 0% to 100%:
 * for (uint16_t pw = 1000; pw <= 2000; pw += 10) {
 *     pwm_set_throttle(pw);
 *     vTaskDelay(pdMS_TO_TICKS(50));
 * }
 */
int32_t pwm_set_throttle(uint16_t pulse_us);

/**
 * @brief Set servo position
 * 
 * Updates servo position on TIM3_CH1 (servo_id=0) or TIM3_CH2 (servo_id=1).
 * Value is clamped to configured min/max range.
 * 
 * @param servo_id 0 for TIM3_CH1 (PA6), 1 for TIM3_CH2 (PA7)
 * @param pulse_us Pulse width in microseconds (1000-2000 typical)
 * 
 * @return 0 on success, -1 if parameters are invalid
 * 
 * @pre pwm_init_servos() has been called
 * @post Servo position updated, range enforced
 * 
 * @example
 * pwm_set_servo(0, 1300);  // Servo 0 deflects left
 * pwm_set_servo(1, 1700);  // Servo 1 deflects right
 */
int32_t pwm_set_servo(uint8_t servo_id, uint16_t pulse_us);

/**
 * @brief Get current servo position
 * 
 * @param servo_id 0 or 1
 * 
 * @return Current pulse width in microseconds, or 0 if invalid servo_id
 */
uint16_t pwm_get_servo(uint8_t servo_id);

/**
 * @brief Get current motor throttle
 * 
 * @return Current throttle pulse width in microseconds
 */
uint16_t pwm_get_throttle(void);

/**
 * @brief Set all servos to neutral position
 * 
 * Useful for initialization and failsafe procedures.
 * 
 * @return 0 on success
 */
int32_t pwm_set_servos_neutral(void);

/**
 * @brief Set motor throttle to minimum (failsafe)
 * 
 * Cuts motor throttle immediately. Used in failsafe conditions.
 * 
 * @return 0 on success
 */
int32_t pwm_set_motor_min(void);

#endif  // PWM_H
