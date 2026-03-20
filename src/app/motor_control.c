#include "FreeRTOS.h"
#include "task.h"
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/timer.h>

#include "mcre7_v2.h"
#include "driver/pwm.h"
#include "motor_control.h"

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
