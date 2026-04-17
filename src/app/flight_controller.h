#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdint.h>

/*********************** Motor Control State ***********************/
typedef int32_t motor_control_error_t;
#define MOTOR_CONTROL_OK                    0
#define MOTOR_CONTROL_INVALID_ARG         (-1)
#define MOTOR_CONTROL_READ_CHANNELS_ERROR (-2)
#define MOTOR_CONTROL_MUTEX_ERROR         (-3)

/*********************** Motor Control Constants ***********************/
#define LEFT_SERVO               1 // Servo on pin PA6
#define RIGHT_SERVO              2 // Servo on pin PA7
#define RECEIVER_MOTOR_MIN_VALUE 60
#define RECEIVER_MOTOR_MAX_VALUE 1608
#define RECEIVER_SERVO_MIN_VALUE 300
#define RECEIVER_SERVO_MAX_VALUE 1900
#define RECEIVER_SERVO_MID_VALUE (RECEIVER_SERVO_MIN_VALUE + (RECEIVER_SERVO_MAX_VALUE - RECEIVER_SERVO_MIN_VALUE) / 2)
#define MIN_PWM_PULSE_WIDTH      1000
#define MAX_PWM_PULSE_WIDTH      2000
#define PWM_PULSE_WIDTH_RANGE    (MAX_PWM_PULSE_WIDTH - MIN_PWM_PULSE_WIDTH)
#define SERVO_OFFSET             1500
#define MAX_SERVO_OUTPUT         500
#define MIN_SERVO_OUTPUT         -500
#define MAX_DESIRED_ANGLE        75.0f

/*************************Motor Control State *************************/
#define FLIGHT_MODE_LOCKED 0
#define FLIGHT_MODE_NORMAL 1
#define FLIGHT_MODE_BALANCING 2

typedef struct {
    uint8_t flight_mode; // 0 = locked, 1 = normal mode, 2 = balancing flight mode
    uint8_t transmitter_powered_on; // 0 = off, 1 = on
    SemaphoreHandle_t mutex;
    uint16_t timeout; // Default timeout of 100 ms for mutex operations
} motor_control_state_t;
extern motor_control_state_t g_motor_control_state;

/*********************** Public API ***********************/
void set_throttle(uint16_t us);
void set_servo(uint8_t channel, uint16_t us);
uint16_t convert_sbus_to_pwm(uint16_t sbus_value);
uint16_t convert_sbus_to_pwm_servo(uint16_t sbus_value);
float convert_sbus_to_angle_roll(uint16_t value);
float convert_sbus_to_angle_pitch(uint16_t value);
void flight_task(void* params);

#endif // MOTOR_CONTROL_H