#include "FreeRTOS.h"
#include "task.h"
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/timer.h>

#include "mcre7_v2.h"
#include "driver/pwm.h"
#include "driver/uart.h"
#include "motor_control.h"
#include "mpu6050.h"
#include "util/pid.h"

void motor_arm(void) {
    for (int i=0; i < 100; i++) {
        set_throttle(1000);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

uint8_t is_motor_locked(void) {
    if(rc_channels[4] != 1800){
        return 1;
    }
    return 0;
}

uint8_t is_transmitter_powered_on(void) {
    if(rc_channels[4] == 0) {
        return 0;
    }
    return 1;
}

void set_throttle(uint16_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    timer_set_oc_value(TIM2, TIM_OC1, us);
}

void set_servo(uint8_t channel, uint16_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;

    if (channel == 1) timer_set_oc_value(TIM3, TIM_OC1, us);
    if (channel == 2) timer_set_oc_value(TIM3, TIM_OC2, us);
}

/* convert from sbus value received from receiver to pwm pulse width*/
uint16_t convert_sbus_to_pwm(uint16_t sbus_value) {
    if (sbus_value < RECEIVER_MOTOR_MIN_VALUE) sbus_value = RECEIVER_MOTOR_MIN_VALUE;
    if (sbus_value > RECEIVER_MOTOR_MAX_VALUE) sbus_value = RECEIVER_MOTOR_MAX_VALUE;
    // linear interpolation: output = out_min + (input - in_min) * (out_range / in_range)
    return MIN_PWM_PULSE_WIDTH + (sbus_value - RECEIVER_MOTOR_MIN_VALUE) + PWM_PULSE_WIDTH_RANGE / (RECEIVER_MOTOR_MAX_VALUE - RECEIVER_MOTOR_MIN_VALUE);
}

uint16_t convert_sbus_to_pwm_servo(uint16_t sbus_value) {
    if (sbus_value < RECEIVER_SERVO_MIN_VALUE) sbus_value = RECEIVER_SERVO_MIN_VALUE;
    if (sbus_value > RECEIVER_SERVO_MAX_VALUE) sbus_value = RECEIVER_SERVO_MAX_VALUE;
    // linear interpolation: output = out_min + (input - in_min) * (out_range / in_range)
    return MIN_PWM_PULSE_WIDTH + (sbus_value - RECEIVER_SERVO_MIN_VALUE) + PWM_PULSE_WIDTH_RANGE / (RECEIVER_SERVO_MAX_VALUE - RECEIVER_SERVO_MIN_VALUE);
}

/* linear mapping sbus to angle: output = (input - center) * (output_range / input_range)*/
float convert_sbus_to_angle(uint16_t value) {
    if (value < RECEIVER_SERVO_MIN_VALUE) value = RECEIVER_SERVO_MIN_VALUE;
    if (value > RECEIVER_SERVO_MAX_VALUE) value = RECEIVER_SERVO_MAX_VALUE;
    return ((int)value - RECEIVER_SERVO_MID_VALUE) * (60.0f) / (RECEIVER_SERVO_MAX_VALUE - RECEIVER_SERVO_MIN_VALUE);
}

void motor_task(void* param) {
    (void)param;
    while(1) {
            uint16_t throttle = convert_sbus_to_pwm(rc_channels[2]);
            uint16_t servo1 = convert_sbus_to_pwm_servo(rc_channels[0]);
            uint16_t servo2 = convert_sbus_to_pwm_servo(rc_channels[1]);
            if (is_transmitter_powered_on()) {
                set_servo(1, servo1);
                set_servo(2, servo2);
            }
            if (!is_motor_locked()) {
                set_throttle(throttle);
            }
            // 50Hz
            vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void pid_task(void* param) {
    (void)param;
    // Outer loop (angle)
    pid_init(&pid_angle_roll,
             3.0f, 0.0f, 0.0f,
             -200.0f, 200.0f,
             -50.0f, 50.0f);
    pid_init(&pid_angle_pitch,
             3.0f, 0.0f, 0.0f,
             -200.0f, 200.0f,
             -50.0f, 50.0f);
    // Inner loop (rate)
    pid_init(&pid_rate_roll,
             3.0f, 0.0f, 0.002f,
             -500.0f, 500.0f,
             -100.0f, 100.0f);
    pid_init(&pid_rate_pitch,
             0.1f, 0.0f, 0.002f,
             -500.0f, 500.0f,
             -100.0f, 100.0f);
    float dt = 0.01f;
    while(1) {
        /* Read state*/
        float roll = angle.angle_roll;
        float pitch = angle.angle_pitch;

        float gyro_roll = physical_data.gyro_x;
        float gyro_pitch = physical_data.gyro_y;

        /* Rx Input*/
        float desired_roll = convert_sbus_to_angle(rc_channels[0]);
        float desired_pitch = convert_sbus_to_angle(rc_channels[1]);
        // uart_printf(">Desired roll:%f,Desired pitch:%f\r\n", desired_roll, desired_pitch);

        /* Outer loop*/
        float desired_roll_rate = pid_update(&pid_angle_roll, desired_roll, roll, dt);
        float desired_pitch_rate = pid_update(&pid_angle_pitch, desired_pitch, pitch, dt);
        // uart_printf(">Desired roll rate:%f,Desired pitch rate:%f\r\n", desired_roll_rate, desired_pitch_rate);

        /* Inner loop*/
        float roll_out = pid_update(&pid_rate_roll, desired_roll_rate, gyro_roll, dt);
        float pitch_out = pid_update(&pid_rate_pitch, 0, gyro_pitch, dt);
        // uart_printf(">PID roll:%f,PID pitch:%f\r\n", roll_out, pitch_out);

        /* Mixing*/
        if (is_transmitter_powered_on()) {
            set_servo(LEFT_SERVO, SERVO_OFFSET + roll_out);
            set_servo(RIGHT_SERVO, SERVO_OFFSET - roll_out);
            uart_printf(">Leftservo:%f,Rightservo:%f\r\n", roll_out + SERVO_OFFSET, SERVO_OFFSET - pitch_out);
        }

        uint16_t throttle = convert_sbus_to_pwm(rc_channels[2]);
        if (!is_motor_locked()) {
            set_throttle(throttle);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}