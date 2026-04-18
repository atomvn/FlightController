/**
 * @file motor_control.c
 * @brief Implementation of motor control logic for flight controller
 *
 * @details
 * Implements motor arming, mode checking, throttle and servo control based on S-BUS input and MPU6050 sensor data.
 * 
 * @author Hao Nguyen
 * @version 1.0
 * @date 2026
 */
#include "FreeRTOS.h"
#include "task.h"
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/timer.h>

#include "mcre7_v2.h"
#include "driver/pwm.h"
#include "driver/uart.h"
#include "flight_controller.h"
#include "mpu6050.h"
#include "util/pid.h"

motor_control_state_t g_motor_control_state = {
    .flight_mode = 0,
    .transmitter_powered_on = 0,
    .take_off = false,
    .mutex = NULL,
    .timeout = pdMS_TO_TICKS(100) // Default timeout of 100 ms for mutex operations
};

static uint8_t takeoff_detection_counter = 0;

typedef enum {
    TAKEOFF_STATE_IDLE,
    TAKEOFF_STATE_DETECTED,
    TAKEOFF_STATE_FINISHED
} takeoff_state_t;

static takeoff_state_t takeoff_state = TAKEOFF_STATE_IDLE;
static TickType_t takeoff_start_time = 0;

/** @brief Arm the motors by setting throttle to minimum for a short duration
 *  @details This function sends a minimum throttle signal for 2 seconds to ensure ESCs are armed. Should be called before enabling flight modes.
 *  @return None
 */
static void motor_arm(void) {
    for (int i=0; i < 100; i++) {
        set_throttle(1000);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/** @brief Initialize the motor control system
 *  @details Initializes motor arming and creates mutex for motor control state. Should be called before starting flight task.
 *  @return Error code indicating success or type of failure
 */
static motor_control_error_t motor_control_init(void) {
    motor_arm();
    g_motor_control_state.mutex = xSemaphoreCreateMutex();
    if (g_motor_control_state.mutex == NULL) {
        return MOTOR_CONTROL_MUTEX_ERROR; // Failed to create mutex
    }
    return MOTOR_CONTROL_OK;
}

/** @brief Update motor control state based on latest S-BUS channel values
 *  @details Reads S-BUS channels to determine flight mode and transmitter power state, and updates global motor control state with thread safety.
 *  @return Error code indicating success or type of failure
 */
static motor_control_error_t update_motor_control_state(void) {
    uint16_t rc_channels[CHANNEL_NUM];
    mcre7_v2_error_t err = read_mcre7_v2_channels(rc_channels, pdMS_TO_TICKS(100));
    if (err != MCRE7_V2_OK) {
        uart_printf("[ERROR] Failed to read MCRE7 V2 channels: %d\n", err);
        return MOTOR_CONTROL_READ_CHANNELS_ERROR; // Keep previous state if we can't read channels
    }
    if (xSemaphoreTake(g_motor_control_state.mutex, g_motor_control_state.timeout) != pdTRUE) {
        return MOTOR_CONTROL_MUTEX_ERROR; // Failed to take mutex, keep previous state
    }
    g_motor_control_state.transmitter_powered_on = (rc_channels[4] != 0);
    if (rc_channels[4] == 1800) {
        g_motor_control_state.flight_mode = FLIGHT_MODE_LOCKED; // Locked
    } else if (rc_channels[4] == 1000) {
        g_motor_control_state.flight_mode = FLIGHT_MODE_NORMAL; // Normal mode
    } else if (rc_channels[4] == 200) {
        g_motor_control_state.flight_mode = FLIGHT_MODE_BALANCING; // Balancing flight mode
    }
    if (rc_channels[5] == 200) {
        g_motor_control_state.take_off = true; // Take off mode
    }
    xSemaphoreGive(g_motor_control_state.mutex);
    return MOTOR_CONTROL_OK;
}

/** @brief Set motor throttle with specified PWM value
 * @param[in] us PWM pulse width in microseconds (1000-2000) 
 * @return None
*/
void set_throttle(uint16_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    timer_set_oc_value(TIM2, TIM_OC1, us);
}

/** @brief Send pwm signal to servo
 * @param[in] channel Servo channel (1 for left servo on PA6, 2 for right servo on PA7)
 * @param[in] us PWM pulse width in microseconds (1000-2000) 
 * @return None
*/
void set_servo(uint8_t channel, uint16_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;

    if (channel == 1) timer_set_oc_value(TIM3, TIM_OC1, us);
    if (channel == 2) timer_set_oc_value(TIM3, TIM_OC2, us);
}

/** @brief convert from sbus motor value received from receiver to PWM value
 *  @param[in] sbus_value S-BUS channel value (typically 60-1608 for motors)
 * return PWM pulse width in microseconds (1000-2000)
*/
uint16_t convert_sbus_to_pwm(uint16_t sbus_value) {
    if (sbus_value < RECEIVER_MOTOR_MIN_VALUE) sbus_value = RECEIVER_MOTOR_MIN_VALUE;
    if (sbus_value > RECEIVER_MOTOR_MAX_VALUE) sbus_value = RECEIVER_MOTOR_MAX_VALUE;
    // linear interpolation: output = out_min + (input - in_min) * (out_range / in_range)
    return MIN_PWM_PULSE_WIDTH + (sbus_value - RECEIVER_MOTOR_MIN_VALUE) * PWM_PULSE_WIDTH_RANGE / (RECEIVER_MOTOR_MAX_VALUE - RECEIVER_MOTOR_MIN_VALUE);
}


/** @brief convert from sbus servo value received from receiver to PWN value
 * @param[in] sbus_value S-BUS channel value (typically 300-1900 for servos)
 * @return PWM pulse width in microseconds (1000-2000)
 */
uint16_t convert_sbus_to_pwm_servo(uint16_t sbus_value) {
    if (sbus_value < RECEIVER_SERVO_MIN_VALUE) sbus_value = RECEIVER_SERVO_MIN_VALUE;
    if (sbus_value > RECEIVER_SERVO_MAX_VALUE) sbus_value = RECEIVER_SERVO_MAX_VALUE;
    // linear interpolation: output = out_min + (input - in_min) * (out_range / in_range)
    return MIN_PWM_PULSE_WIDTH + (sbus_value - RECEIVER_SERVO_MIN_VALUE) * PWM_PULSE_WIDTH_RANGE / (RECEIVER_SERVO_MAX_VALUE - RECEIVER_SERVO_MIN_VALUE);
}

/* */
/** @brief linear mapping sbus to angle: output = (input - center) * (output_range / input_range)
 *  @param[in] sbus_value S-BUS channel value (typically 300-1900 for servos)
 *  @return Desired angle in degrees (-75 to 75)
*/
float convert_sbus_to_angle_roll(uint16_t value) {
    if (value < RECEIVER_SERVO_MIN_VALUE) value = RECEIVER_SERVO_MIN_VALUE;
    if (value > RECEIVER_SERVO_MAX_VALUE) value = RECEIVER_SERVO_MAX_VALUE;
    return ((int)value - RECEIVER_SERVO_MID_VALUE) * (-2*MAX_DESIRED_ANGLE) / (RECEIVER_SERVO_MAX_VALUE - RECEIVER_SERVO_MIN_VALUE);
}

float convert_sbus_to_angle_pitch(uint16_t value) {
    if (value < RECEIVER_SERVO_MIN_VALUE) value = RECEIVER_SERVO_MIN_VALUE;
    if (value > RECEIVER_SERVO_MAX_VALUE) value = RECEIVER_SERVO_MAX_VALUE;
    return ((int)value - RECEIVER_SERVO_MID_VALUE) * (2*MAX_DESIRED_ANGLE) / (RECEIVER_SERVO_MAX_VALUE - RECEIVER_SERVO_MIN_VALUE);
}

/** @brief Clamp servo output values
 *  @param[in,out] servo_total_output Pointer to the servo output value to be clamped
 */
static void servo_output_clamp(float* servo_total_output) {
    if (*servo_total_output > MAX_SERVO_OUTPUT) {
        *servo_total_output = MAX_SERVO_OUTPUT;
    }
    if (*servo_total_output < MIN_SERVO_OUTPUT) {
        *servo_total_output = MIN_SERVO_OUTPUT;
    }
}

/** @brief Detect takeoff condition based on MPU6050 sensor data
 *  @details Checks if acceleratio sum exceeds a defined threshold for a certain number of consecutive samples to confirm takeoff condition.
 *  @return true if takeoff condition is detected, false otherwise
 */
static bool detect_takeoff_condition(mpu6050_t *current_mpu6050_data) {
    float acceleration_sum = current_mpu6050_data->physical_data.accel_x + current_mpu6050_data->physical_data.accel_y + current_mpu6050_data->physical_data.accel_z;
    uart_printf("[LOG] Acceleration sum: %f", acceleration_sum);
    
    if (acceleration_sum > TAKEOFF_ACCEL_THRESHOLD) {
        takeoff_detection_counter++;
        if (takeoff_detection_counter >= TAKEOFF_DETECTION_SAMPLES) {
            takeoff_detection_counter = 0; // Reset counter after confirming takeoff
            uart_printf("[LOG] Takeoff condition detected!");
            return true; // Detected takeoff condition (significant drop in vertical acceleration)
        }
    }
    else {
        takeoff_detection_counter = 0; // Reset counter if condition is not met
    }
    return false; // Not takeoff condition
}

/** @brief Main flight control task
 *  @details Implements the main control loop for the flight controller, including reading sensor data, calculating PID outputs, and setting motor/servo outputs based on flight mode and S-BUS input.
 *  @param[in] params Task parameters (not used)
 *  @return None
 */
void flight_task(void* params) {
    (void)params;
    motor_control_error_t init_err = motor_control_init();
    if (init_err != MOTOR_CONTROL_OK) {
        uart_printf("[ERROR] Motor control initialization failed with error code: %d\n", init_err);
        while(1);
     }
    // Outer loop (angle)
    pid_init(&pid_angle_roll,
             4.0f, 0.0f, 0.0f,
             -500.0f, 500.0f,
             -50.0f, 50.0f);
    pid_init(&pid_angle_pitch,
             4.0f, 0.0f, 0.0f,
             -500.0f, 500.0f,
             -50.0f, 50.0f);
    // Inner loop (rate)
    pid_init(&pid_rate_roll,
             2.0f, 0.01f, 0.001f,
             -500.0f, 500.0f,
             -100.0f, 100.0f);
    pid_init(&pid_rate_pitch,
             2.0f, 0.01f, 0.001f,
             -500.0f, 500.0f,
             -100.0f, 100.0f);
    float dt = 0.01f;
    while(1) {
        motor_control_error_t motor_control_err = update_motor_control_state();
        if (motor_control_err != MOTOR_CONTROL_OK) {
            uart_printf("[ERROR] Failed to update motor control state: %d\n", motor_control_err);
        }

        uint16_t rc_channels[CHANNEL_NUM];
        mcre7_v2_error_t mcre_err = read_mcre7_v2_channels(rc_channels, pdMS_TO_TICKS(100));
        if (mcre_err != MCRE7_V2_OK) {
            uart_printf("[ERROR] Failed to read MCRE7 V2 channels: %d\n", mcre_err);
        }

        mpu6050_t current_mpu6050_data;
        mpu6050_error_t mpu_err = read_mpu6050_data(&current_mpu6050_data);
        if (mpu_err != MPU6050_OK) {
            uart_printf("[ERROR] Failed to read MPU6050 data: %d\n", mpu_err);
        }

        /* Take off mode*/
        if (takeoff_state == TAKEOFF_STATE_IDLE && g_motor_control_state.take_off == true) {
            uart_printf("[LOG] Take_off: %d\n", g_motor_control_state.take_off);
            if (true == detect_takeoff_condition(&current_mpu6050_data)) {
                takeoff_state = TAKEOFF_STATE_DETECTED;
                takeoff_start_time = xTaskGetTickCount();
                uart_printf("[LOG] Takeoff condition detected, starting takeoff sequence\n");
                while(xTaskGetTickCount() - takeoff_start_time < TAKEOFF_DURATION) { // Run takeoff sequence for 5 seconds
                    motor_control_error_t motor_control_err = update_motor_control_state();
                    if (motor_control_err != MOTOR_CONTROL_OK) {
                    uart_printf("[ERROR] Failed to update motor control state: %d\n", motor_control_err);
                    }

                    uart_printf("[LOG] Taking off..............\n");
                    float desired_roll = 0.0f; // Keep level during takeoff
                    float desired_pitch = TAKEOFF_PITCH_THRESHOLD;

                    float roll = current_mpu6050_data.angle_roll;
                    float pitch = current_mpu6050_data.angle_pitch;

                    float gyro_roll = current_mpu6050_data.physical_data.gyro_y;
                    float gyro_pitch = current_mpu6050_data.physical_data.gyro_x;

                    /* Outer loop*/
                    float desired_roll_rate = pid_update(&pid_angle_roll, desired_roll, roll, dt);
                    float desired_pitch_rate = pid_update(&pid_angle_pitch, desired_pitch, pitch, dt);
                    // uart_printf(">Desired roll rate:%f,Desired pitch rate:%f\r\n", desired_roll_rate, desired_pitch_rate);

                    /* Inner loop*/
                    float roll_out = pid_update(&pid_rate_roll, desired_roll_rate, gyro_roll, dt);
                    float pitch_out = pid_update(&pid_rate_pitch, desired_pitch_rate, gyro_pitch, dt);
                    float pid_left_servo_out = roll_out - pitch_out;
                    float pid_right_servo_out = roll_out + pitch_out;
                    servo_output_clamp(&pid_left_servo_out);
                    servo_output_clamp(&pid_right_servo_out);

                    // uart_printf(">PID roll:%f,PID pitch:%f\r\n", roll_out, pitch_out);

                    /* Mixing*/
                    if (g_motor_control_state.transmitter_powered_on) {
                        set_servo(LEFT_SERVO, SERVO_OFFSET + pid_left_servo_out);
                        set_servo(RIGHT_SERVO, SERVO_OFFSET + pid_right_servo_out);
                    }
                
                    if (g_motor_control_state.flight_mode != FLIGHT_MODE_LOCKED) {
                        set_throttle(TAKEOFF_THROTTLE);
                    } else {
                        set_throttle(MIN_PWM_PULSE_WIDTH);
                    }

                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                takeoff_state = TAKEOFF_STATE_FINISHED;
                uart_printf("[LOG] Takeoff sequence finished, switching to normal flight mode\n");
            }
        }

        /* Balancing flight mode */
        if (g_motor_control_state.flight_mode == FLIGHT_MODE_BALANCING) { 
            /* Read state*/
            float roll = current_mpu6050_data.angle_roll;
            float pitch = current_mpu6050_data.angle_pitch;

            // float gyro_roll = physical_data.gyro_x;
            // float gyro_pitch = physical_data.gyro_y;
            float gyro_roll = current_mpu6050_data.physical_data.gyro_y;
            float gyro_pitch = current_mpu6050_data.physical_data.gyro_x;

            /* Rx Input*/
            float desired_roll = convert_sbus_to_angle_roll(rc_channels[0]);
            float desired_pitch = convert_sbus_to_angle_pitch(rc_channels[1]);
            // uart_printf(">Desired roll:%f,Desired pitch:%f\r\n", desired_roll, desired_pitch);

            /* Outer loop*/
            float desired_roll_rate = pid_update(&pid_angle_roll, desired_roll, roll, dt);
            float desired_pitch_rate = pid_update(&pid_angle_pitch, desired_pitch, pitch, dt);
            // uart_printf(">Desired roll rate:%f,Desired pitch rate:%f\r\n", desired_roll_rate, desired_pitch_rate);

            /* Inner loop*/
            float roll_out = pid_update(&pid_rate_roll, desired_roll_rate, gyro_roll, dt);
            float pitch_out = pid_update(&pid_rate_pitch, desired_pitch_rate, gyro_pitch, dt);
            float pid_left_servo_out = roll_out - pitch_out;
            float pid_right_servo_out = roll_out + pitch_out;
            servo_output_clamp(&pid_left_servo_out);
            servo_output_clamp(&pid_right_servo_out);

            // uart_printf(">PID roll:%f,PID pitch:%f\r\n", roll_out, pitch_out);

            /* Mixing*/
            if (g_motor_control_state.transmitter_powered_on) {
                set_servo(LEFT_SERVO, SERVO_OFFSET + pid_left_servo_out);
                set_servo(RIGHT_SERVO, SERVO_OFFSET + pid_right_servo_out);
            }
           
            uint16_t throttle = convert_sbus_to_pwm(rc_channels[2]);
            if (g_motor_control_state.flight_mode != FLIGHT_MODE_LOCKED) {
                set_throttle(throttle);
            }            
            vTaskDelay(pdMS_TO_TICKS(10));

        /* Normal flight mode */
        } else if (g_motor_control_state.flight_mode == FLIGHT_MODE_NORMAL) { 
            uint16_t throttle = convert_sbus_to_pwm(rc_channels[2]);
            float  desired_roll = (50/4.5f) * convert_sbus_to_angle_roll(rc_channels[0]);
            float desired_pitch = (50/4.5f) * convert_sbus_to_angle_pitch(rc_channels[1]);
            // uart_printf("desired_roll: %f, desired_pitch: %f\n", desired_roll, desired_pitch);

            float left_servo_out = desired_roll - desired_pitch;
            float right_servo_out = desired_roll + desired_pitch;
            // uart_printf("left_servo_out: %f, right_servo_out: %f\n", left_servo_out, right_servo_out);

            servo_output_clamp(&left_servo_out);
            servo_output_clamp(&right_servo_out);

            if (g_motor_control_state.transmitter_powered_on) {
                set_servo(LEFT_SERVO,SERVO_OFFSET + left_servo_out);
                set_servo(RIGHT_SERVO, SERVO_OFFSET + right_servo_out);
            }
            if (g_motor_control_state.flight_mode != FLIGHT_MODE_LOCKED) {
                set_throttle(throttle);
            }            // 50Hz
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else { // Locked mode
            set_throttle(convert_sbus_to_pwm(RECEIVER_MOTOR_MIN_VALUE));
            set_servo(LEFT_SERVO, SERVO_OFFSET);
            set_servo(RIGHT_SERVO, SERVO_OFFSET);
        }
    }
}