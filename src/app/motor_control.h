#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdint.h>

#define LEFT_SERVO 1
#define RIGHT_SERVO 2

static const uint16_t RECEIVER_MOTOR_MIN_VALUE = 200;
static const uint16_t RECEIVER_MOTOR_MAX_VALUE = 1608;
static const uint16_t RECEIVER_SERVO_MIN_VALUE = 600;
static const uint16_t RECEIVER_SERVO_MAX_VALUE = 1400;
static const uint16_t RECEIVER_SERVO_MID_VALUE = RECEIVER_SERVO_MIN_VALUE + (RECEIVER_SERVO_MAX_VALUE - RECEIVER_SERVO_MIN_VALUE) / 2;
static const uint16_t MIN_PWM_PULSE_WIDTH = 1000;
static const uint16_t MAX_PWM_PULSE_WIDTH = 2000;
static const uint16_t PWM_PULSE_WIDTH_RANGE = MAX_PWM_PULSE_WIDTH - MIN_PWM_PULSE_WIDTH;
static const uint16_t SERVO_OFFSET = 1500;
static const float KP = 3.0f;
static const float KI = 0.0f;
static const float KD = 0.0f;
static const float PID_OUT_MIN = -200.0f;
static const float PID_OUT_MAX = 200.0f;
static const float INTEGRAL_MIN = -50.0f;
static const float INTEGRAL_MAX = 50.0f;

typedef struct {
    float kp, ki, kd;

    float integral;
    float prev_error;

    float integral_min;
    float integral_max;

    float out_min, out_max;

} PID_t;

PID_t pid_angle_roll;
PID_t pid_angle_pitch;
PID_t pid_rate_roll;
PID_t pid_rate_pitch;

void motor_arm(void);
uint8_t is_motor_locked(void);
uint8_t is_transmitter_powered_on(void);
void set_throttle(uint16_t us);
void set_servo(uint8_t channel, uint16_t us);
uint16_t convert_sbus_to_pwm(uint16_t sbus_value);
uint16_t convert_sbus_to_pwm_servo(uint16_t sbus_value);
float convert_sbus_to_angle(uint16_t value);
// void pid_init(PID_t* pid);
void pid_init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max, float integral_min, float integral_max);
float pid_update(PID_t* pid, float setpoint, float measurement, float dt);
void motor_task(void* param);
void pid_task(void* param);

#endif