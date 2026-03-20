#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdint.h>

static const uint16_t RECEIVER_MOTOR_MIN_VALUE = 200;
static const uint16_t RECEIVER_MOTOR_MAX_VALUE = 1608;
static const uint16_t RECEIVER_SERVO_MIN_VALUE = 536;
static const uint16_t RECEIVER_SERVO_MAX_VALUE = 1336;
static const uint16_t MIN_PWM_PULSE_WIDTH = 1000;
static const uint16_t MAX_PWM_PULSE_WIDTH = 2000;
static const uint16_t PWM_PULSE_WIDTH_RANGE = MAX_PWM_PULSE_WIDTH - MIN_PWM_PULSE_WIDTH;
static const uint16_t SERVO_OFFSET = 1000;

void motor_arm(void);
uint8_t is_motor_locked(void);
uint8_t is_transmitter_powered_on(void);
void set_throttle(uint16_t us);
void set_servo(uint8_t channel, uint16_t us);
uint16_t convert_sbus_to_pwm(uint16_t sbus_value);
uint16_t convert_sbus_to_pwm_servo(uint16_t sbus_value);
void motor_task(void* param);

#endif