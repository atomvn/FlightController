/**
 * @file mpu6050.c
 * @brief MPU6050 IMU Driver Implementation
 *
 * @details
 * Implement MPU6050 task which reads accelerometer and gyroscope data over I2C, converts to physical units, and calculates angles.
 * Provides API for MPU6050 initialization, data reading, and angle calculation with thread safety.
 * 
 * @author Hao Nguyen
 * @version 1.0
 * @date 2026
 */
#include <math.h>
#include "FreeRTOS.h"
#include <task.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/i2c.h>
#include "libopencm3/stm32/f1/gpio.h"
#include "libopencm3/stm32/f1/usart.h"

#include "mpu6050.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "util/sensor_fusion.h"

#define IS_MPU6050_DEBUG_ENABLED 1

mpu6050_t g_mpu6050 = {
    .angle_roll = 0.0f,
    .angle_pitch = 0.0f,
    .raw_data = {0},
    .physical_data = {0.0f},
    .mutex = NULL,
    .timeout = pdMS_TO_TICKS(100) // Default timeout of 100 ms for mutex operations
};

/* MPU6050 I2C */
static I2C_control i2c;

/* MPU6050 angle calculation based on accelerometer variables */
typedef struct {
    float angle_roll;
    float angle_pitch;
} mpu6050_angle_acc_t;

/* MPU6050 calibrating offsets */
typedef struct {
    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;
} mpu6050_calibrating_offset_t;

static mpu6050_angle_acc_t angle_acc; 
static mpu6050_calibrating_offset_t offsets;

/** @brief Write data to a specific register on the MPU6050
 *  @param[in] reg Register address
 *  @param[in] data Data to write
 *  @return Error code
 */
static mpu6050_error_t mpu6050_write_to_reg(uint8_t reg, uint8_t data) {
    if (reg > 0x7F) {
        return MPU6050_ERR_I2C; // Invalid register address
    }
    if (i2c_start_addr(&i2c, MPU6050_ADDR, I2C_WRITE) != I2C_OK) {
        return MPU6050_ERR_I2C; // Failed to start I2C communication
    }

    if (i2c_write(&i2c, reg) != I2C_OK) {
        i2c_send_stop(i2c.device);
        return MPU6050_ERR_I2C; // Failed to write register address
    }
    if (i2c_write(&i2c, data) != I2C_OK) {
        i2c_send_stop(i2c.device);
        return MPU6050_ERR_I2C; // Failed to write data
    }
    i2c_send_stop(i2c.device);
    return MPU6050_OK;
}

/** @brief Initialize the MPU6050 sensor and its I2C communication
 *  @return Error code
 */
static mpu6050_error_t mpu6050_init(void) {
    if (i2c_init(&i2c, I2C1, &I2C_CONFIG_400KHZ) != I2C_OK) {
        uart_printf("[ERROR] Failed to initialize I2C for MPU6050\n");
        return MPU6050_ERR_I2C;
    }
    if (mpu6050_write_to_reg(MPU6050_PWR_MGMT_1, 0x00) != MPU6050_OK) {
        uart_printf("[ERROR] Failed to wake up MPU6050\n");
        return MPU6050_ERR_I2C;
    }
        
    if (mpu6050_write_to_reg(MPU6050_SMPLRT_DIV, 0x07)  != MPU6050_OK
     || mpu6050_write_to_reg(MPU6050_GYRO_CFG,   0x10)  != MPU6050_OK
     || mpu6050_write_to_reg(MPU6050_ACCEL_CFG,  0x10)  != MPU6050_OK
     || mpu6050_write_to_reg(MPU6050_CONFIG,     0x03)  != MPU6050_OK) {
        uart_printf("[ERROR] Failed to configure MPU6050\n");
        return MPU6050_ERR_I2C;
    }
    g_mpu6050.mutex = xSemaphoreCreateMutex();
    return MPU6050_OK;
}

/** @brief Read accelerometer and gyroscope data from the MPU6050
 *  @param[out] data Pointer to structure to store raw data
 *  @return Error code
 */
static mpu6050_error_t mpu6050_read_accel_gyro(mpu6050_raw_data_t *data) {
    if (data == NULL) {
        return MPU6050_ERR_I2C; // Invalid output pointer
    }

    if (i2c_start_addr(&i2c, MPU6050_ADDR, I2C_WRITE) != I2C_OK) {
        return MPU6050_ERR_I2C; // Failed to start I2C communication
    }
    if (i2c_write(&i2c, MPU6050_ACCEL_XH) != I2C_OK) {
        i2c_send_stop(i2c.device);
        return MPU6050_ERR_I2C; // Failed to write register address
    }
    i2c_send_stop(i2c.device);
    if (i2c_start_addr(&i2c, MPU6050_ADDR, I2C_READ) != I2C_OK) {
        return MPU6050_ERR_I2C; // Failed to start I2C communication for reading
    }
    uint8_t raw[14];
    for (int i = 0; i < 14; i++) {
        i2c_status_t status = i2c_read(&i2c, i == 13, &raw[i]);
        if (status != I2C_OK) {
            uart_printf("[ERROR] Failed to read MPU6050 data: %s\n", i2c_error_string(status));
            i2c_send_stop(i2c.device);
            return MPU6050_ERR_I2C;
        }
    }
    i2c_send_stop(i2c.device);

    if (xSemaphoreTake(g_mpu6050.mutex, g_mpu6050.timeout) != pdTRUE) {
        uart_printf("[ERROR] Failed to take mutex for MPU6050 data access\n");
        return MPU6050_MUTEX_ERROR; // Failed to take mutex
    }
    /* Combine high and low bytes to form 16-bit signed integers */
    data->accel_x = (int16_t)((raw[0] << 8) | raw[1]);
    data->accel_y = (int16_t)((raw[2] << 8) | raw[3]);
    data->accel_z = (int16_t)((raw[4] << 8) | raw[5]);
    data->gyro_x  = (int16_t)((raw[8] << 8) | raw[9]);
    data->gyro_y  = (int16_t)((raw[10] << 8) | raw[11]);
    data->gyro_z  = (int16_t)((raw[12] << 8) | raw[13]);
    xSemaphoreGive(g_mpu6050.mutex);
    return MPU6050_OK;
}

/** @brief Convert raw sensor data to physical units
 *  @param[in] raw_data Pointer to raw data structure
 *  @param[out] physical_data Pointer to structure to store physical data
 *  @return Error code
 */
static mpu6050_error_t convert_raw_to_physical(mpu6050_raw_data_t *raw_data, mpu6050_physical_data_t *physical_data) {
    if (raw_data == NULL || physical_data == NULL) {
        return MPU6050_ERR_I2C; // Invalid input pointers
    }
    if (xSemaphoreTake(g_mpu6050.mutex, g_mpu6050.timeout) != pdTRUE) {
        return MPU6050_MUTEX_ERROR; // Failed to take mutex
    }
    /* Convert raw values to physical units based on MPU6050 sensitivity settings */
    physical_data->accel_x = (raw_data->accel_x / 4096.0f);
    physical_data->accel_y = (raw_data->accel_y / 4096.0f);
    physical_data->accel_z = (raw_data->accel_z / 4096.0f);
    physical_data->gyro_x  = raw_data->gyro_x / 32.8f;
    physical_data->gyro_y  = raw_data->gyro_y / 32.8f;
    physical_data->gyro_z  = raw_data->gyro_z / 32.8f;
    xSemaphoreGive(g_mpu6050.mutex);
    return MPU6050_OK;
}

/** @brief Calculate roll and pitch angles from accelerometer data
 *  @param[out] angle Pointer to structure to store calculated angles
 *  @return Error code
 */
static mpu6050_error_t calculate_mpu6050_angle_acc(mpu6050_angle_acc_t *angle)
{
    if (angle == NULL) {
        return MPU6050_ERR_I2C; // Invalid output pointer
    }
    /* For my MPU6050, roll and pitch angle are inverted (may be due to hardware fault) so equations to calculate roll and pitch angle are also inverted here to satisfy for that error*/
    if (xSemaphoreTake(g_mpu6050.mutex, g_mpu6050.timeout) != pdTRUE) {
        return MPU6050_MUTEX_ERROR; // Failed to take mutex
    }
    angle->angle_roll = -atan(g_mpu6050.physical_data.accel_x / sqrt(g_mpu6050.physical_data.accel_y*g_mpu6050.physical_data.accel_y + g_mpu6050.physical_data.accel_z*g_mpu6050.physical_data.accel_z)) * RAD_TO_DEG;
    angle->angle_pitch = atan(g_mpu6050.physical_data.accel_y / sqrt(g_mpu6050.physical_data.accel_x*g_mpu6050.physical_data.accel_x + g_mpu6050.physical_data.accel_z*g_mpu6050.physical_data.accel_z)) * RAD_TO_DEG;
    xSemaphoreGive(g_mpu6050.mutex);
    // uart_printf(">Angle acc roll:%f,Angle acc pitch:%f\r\n", angle->angle_roll, angle->angle_pitch);
    return MPU6050_OK;
}

/** @brief Calculate calibration offsets for the MPU6050 gyroscope
 *  @param[out] offsets Pointer to structure to store calculated offsets
 *  @return Error code
 */
static mpu6050_error_t calculate_calibrate_offset(mpu6050_calibrating_offset_t *offsets) {
    uart_printf("[LOG] Starting MPU6050 gyro calibration...\n");
    const int samples = 100;
    mpu6050_calibrating_offset_t offset_sum = {0};

    for (int i = 0; i < samples; i++) {
        uart_printf("[LOG] Collecting calibration sample %d/%d\n", i + 1, samples);
        if (mpu6050_read_accel_gyro(&g_mpu6050.raw_data) != MPU6050_OK) {
            uart_printf("[ERROR] Failed to read MPU6050 data for calibration\n");
            return MPU6050_ERR_I2C;
        }
        uart_printf("[LOG] Raw gyro data: X=%d, Y=%d, Z=%d\n", g_mpu6050.raw_data.gyro_x, g_mpu6050.raw_data.gyro_y, g_mpu6050.raw_data.gyro_z);
        if (convert_raw_to_physical(&g_mpu6050.raw_data, &g_mpu6050.physical_data) != MPU6050_OK) {
            uart_printf("[ERROR] Failed to convert MPU6050 data for calibration\n");
            return MPU6050_ERR_I2C;
        }
        offset_sum.gyro_x_offset  += g_mpu6050.physical_data.gyro_x;
        offset_sum.gyro_y_offset  += g_mpu6050.physical_data.gyro_y;
        offset_sum.gyro_z_offset  += g_mpu6050.physical_data.gyro_z;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    offsets->gyro_x_offset  = offset_sum.gyro_x_offset / samples;
    offsets->gyro_y_offset  = offset_sum.gyro_y_offset / samples;
    offsets->gyro_z_offset  = offset_sum.gyro_z_offset / samples;
    return MPU6050_OK;
}

/** @brief Apply calibration offsets to the MPU6050 sensor data
 *  @param[in,out] data Pointer to physical data structure to apply offsets to
 *  @param[in] offsets Pointer to structure containing calibration offsets
 *  @return Error code
 */
static void apply_calibration(mpu6050_physical_data_t *data, mpu6050_calibrating_offset_t *offsets) {
    data->accel_x -= MANUAL_ACCELX_CALIB_VALUE ;
    data->accel_y -= MANUAL_ACCELY_CALIB_VALUE ;
    data->accel_z -= MANUAL_ACCELZ_CALIB_VALUE ;
    data->gyro_x  -= offsets->gyro_x_offset;
    data->gyro_y  -= offsets->gyro_y_offset;
    data->gyro_z  -= offsets->gyro_z_offset;
}

/** @brief Read MPU6050 sensor data with thread safety
 *  @param[out] data Pointer to structure to store current MPU6050 state
 *  @return Error code
 */
mpu6050_error_t read_mpu6050_data(mpu6050_t* data) {
    if (data == NULL) {
        return MPU6050_INVALID_ARG; // Invalid input pointer
    }
    if (xSemaphoreTake(g_mpu6050.mutex, g_mpu6050.timeout) != pdTRUE) {
        return MPU6050_MUTEX_ERROR; // Failed to take mutex
    }
    *data = g_mpu6050; // Copy current state to output
    xSemaphoreGive(g_mpu6050.mutex);
    return MPU6050_OK;
}

/** @brief Main task for handling MPU6050 sensor operations
 *  @param[in] params Task parameters (not used)
 *  @return None
 */
void mpu6050_task(void *params) {
    (void)params;
    mpu6050_error_t err = mpu6050_init();
    uart_send_string("[LOG] [mpu6050_task] Done MPU6050 init\n");
    if (err != MPU6050_OK) {
        uart_printf("[ERROR] MPU6050 initialization failed with error code: %d\n", err);
    }
    calculate_calibrate_offset(&offsets);
    uart_printf("[LOG] calculated gyro offsets: X=%f, Y=%f, Z=%f\n", offsets.gyro_x_offset, offsets.gyro_y_offset, offsets.gyro_z_offset);
    // calculate_mpu6050_angle_acc(&angle_acc);
    kalman_init(&kalman_roll);
    kalman_init(&kalman_pitch);
    kalman_roll.angle = g_mpu6050.angle_roll;
    kalman_pitch.angle = g_mpu6050.angle_pitch;
    TickType_t last = xTaskGetTickCount();
    while (1) {
        mpu6050_read_accel_gyro(&g_mpu6050.raw_data);
        convert_raw_to_physical(&g_mpu6050.raw_data, &g_mpu6050.physical_data);
        apply_calibration(&g_mpu6050.physical_data, &offsets);
        calculate_mpu6050_angle_acc(&angle_acc);
        TickType_t now = xTaskGetTickCount();
        float dt = (now - last) * portTICK_PERIOD_MS / 1000.0f;
        last = now;
        // gyro_y for roll and x for pitch as for my MPU6050, roll and pitch are inverted
        g_mpu6050.angle_roll = kalman_update(&kalman_roll, angle_acc.angle_roll, g_mpu6050.physical_data.gyro_y, dt);
        g_mpu6050.angle_pitch = kalman_update(&kalman_pitch, angle_acc.angle_pitch, g_mpu6050.physical_data.gyro_x, dt);
        #if IS_MPU6050_DEBUG_ENABLED
        uart_printf(">Angle roll:%f,Angle pitch:%f\r\n", g_mpu6050.angle_roll, g_mpu6050.angle_pitch);
        #endif
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}
