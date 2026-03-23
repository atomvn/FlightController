#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdint.h>

#define LEFT_SERVO 1
#define RIGHT_SERVO 2
#define RECEIVER_MOTOR_MIN_VALUE 200
#define RECEIVER_MOTOR_MAX_VALUE 1608
#define RECEIVER_SERVO_MIN_VALUE 600
#define RECEIVER_SERVO_MAX_VALUE 1400
#define RECEIVER_SERVO_MID_VALUE (RECEIVER_SERVO_MIN_VALUE + (RECEIVER_SERVO_MAX_VALUE - RECEIVER_SERVO_MIN_VALUE) / 2)
#define MIN_PWM_PULSE_WIDTH 1000
#define MAX_PWM_PULSE_WIDTH 2000
#define PWM_PULSE_WIDTH_RANGE (MAX_PWM_PULSE_WIDTH - MIN_PWM_PULSE_WIDTH)
#define SERVO_OFFSET 1500

void motor_arm(void);
uint8_t is_motor_locked(void);
uint8_t is_transmitter_powered_on(void);
void set_throttle(uint16_t us);
void set_servo(uint8_t channel, uint16_t us);
uint16_t convert_sbus_to_pwm(uint16_t sbus_value);
uint16_t convert_sbus_to_pwm_servo(uint16_t sbus_value);
float convert_sbus_to_angle(uint16_t value);
void motor_task(void* param);
void pid_task(void* param);

#endif