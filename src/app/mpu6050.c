#include <math.h>
#include "FreeRTOS.h"
#include <task.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/i2c.h>
#include "libopencm3/stm32/f1/gpio.h"
#include "libopencm3/stm32/f1/usart.h"

#include "mpu6050.h"
#include "driver/uart.h"
#include "util/sensor_fusion.h"

void mpu6050_write_to_reg(uint8_t reg, uint8_t data) {
    i2c_start_addr(&i2c, MPU6050_ADDR, I2C_WRITE);
    i2c_write(&i2c, reg);
    i2c_write(&i2c, data);
    i2c_send_stop(i2c.device);
}

void mpu6050_init() {
    i2c_configure(&i2c, I2C1, 100);
    mpu6050_write_to_reg(MPU6050_PWR_MGMT_1, 0x00); // Wake up the sensor
    mpu6050_write_to_reg(MPU6050_SMPLRT_DIV, 0x07); 
    mpu6050_write_to_reg(MPU6050_GYRO_CFG, 0x10); // measure range +- 1000 degree/s, raw_data / 32.8
    mpu6050_write_to_reg(MPU6050_ACCEL_CFG, 0x10); // measure range +- 8g, raw_data / 4096
    mpu6050_write_to_reg(MPU6050_CONFIG, 0x03); // low pass filter 44 Hz
}

void mpu6050_read_accel_gyro(mpu6050_raw_data *data) {
    i2c_start_addr(&i2c, MPU6050_ADDR, I2C_WRITE);
    i2c_write(&i2c, MPU6050_ACCEL_XH);
    i2c_send_stop(i2c.device);
    i2c_start_addr(&i2c, MPU6050_ADDR, I2C_READ);
    uint8_t raw[14];
    for (int i = 0; i < 14; i++) {
        raw[i] = i2c_read(&i2c, i == 13);
    }
    i2c_send_stop(i2c.device);

    data->accel_x = (int16_t)((raw[0] << 8) | raw[1]);
    data->accel_y = (int16_t)((raw[2] << 8) | raw[3]);
    data->accel_z = (int16_t)((raw[4] << 8) | raw[5]);
    data->gyro_x  = (int16_t)((raw[8] << 8) | raw[9]);
    data->gyro_y  = (int16_t)((raw[10] << 8) | raw[11]);
    data->gyro_z  = (int16_t)((raw[12] << 8) | raw[13]);
}

void convert_raw_to_physical(mpu6050_raw_data *raw_data, mpu6050_physical_data *physical_data) {
    physical_data->accel_x = (raw_data->accel_x / 4096.0f);
    physical_data->accel_y = (raw_data->accel_y / 4096.0f);
    physical_data->accel_z = (raw_data->accel_z / 4096.0f);
    physical_data->gyro_x  = raw_data->gyro_x / 32.8f;
    physical_data->gyro_y  = raw_data->gyro_y / 32.8f;
    physical_data->gyro_z  = raw_data->gyro_z / 32.8f;
}

// void calculate_mpu6050_angle_acc(mpu6050_angle* angle_acc) {
//     angle_acc->angle_roll = atan(physical_data.accel_y / sqrt(physical_data.accel_x*physical_data.accel_x + physical_data.accel_z*physical_data.accel_z))*RAD_TO_DEG;
//     angle_acc->angle_pitch = atan(physical_data.accel_x / sqrt(physical_data.accel_y*physical_data.accel_y + physical_data.accel_z*physical_data.accel_z))*RAD_TO_DEG;
// }

void calculate_mpu6050_angle_acc(
    mpu6050_angle *angle)
{
    angle->angle_roll = atan2f(physical_data.accel_y, physical_data.accel_z) * RAD_TO_DEG;
    angle->angle_pitch = atan2f(-physical_data.accel_x, sqrtf(physical_data.accel_y*physical_data.accel_y + physical_data.accel_z*physical_data.accel_z)) * RAD_TO_DEG;
    uart_printf(">Angle acc roll:%f,Angle acc pitch:%f\r\n", angle->angle_roll, angle->angle_pitch);
}

void calculate_calibrate_offset(mpu6050_calibrating_offset *offsets) {
    const int samples = 100;
    mpu6050_calibrating_offset offset_sum = {0};

    for (int i = 0; i < samples; i++) {
        mpu6050_read_accel_gyro(&data);
        convert_raw_to_physical(&data, &physical_data);
        offset_sum.gyro_x_offset  += physical_data.gyro_x;
        offset_sum.gyro_y_offset  += physical_data.gyro_y;
        offset_sum.gyro_z_offset  += physical_data.gyro_z;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    offsets->gyro_x_offset  = offset_sum.gyro_x_offset / samples;
    offsets->gyro_y_offset  = offset_sum.gyro_y_offset / samples;
    offsets->gyro_z_offset  = offset_sum.gyro_z_offset / samples;
}

void apply_calibration(mpu6050_physical_data *data, mpu6050_calibrating_offset *offsets) {
    data->gyro_x  -= offsets->gyro_x_offset;
    data->gyro_y  -= offsets->gyro_y_offset;
    data->gyro_z  -= offsets->gyro_z_offset;
}

void mpu6050_task(void *params) {
    (void)params;
    mpu6050_calibrating_offset offsets;
    mpu6050_init();
    calculate_calibrate_offset(&offsets);
    kalman_init(&kalman_roll);
    kalman_init(&kalman_pitch);
    TickType_t last = xTaskGetTickCount();
    // uart_send_string("[LOG] [mpu6050_task] Done MPU6050 init\n");
    while (1) {
        mpu6050_read_accel_gyro(&data);
        convert_raw_to_physical(&data, &physical_data);
        apply_calibration(&physical_data, &offsets);
        calculate_mpu6050_angle_acc(&angle_acc);
        TickType_t now = xTaskGetTickCount();
        float dt = (now - last) * portTICK_PERIOD_MS / 1000.0f;
        last = now;
        angle.angle_roll = kalman_update(&kalman_roll, angle_acc.angle_roll, physical_data.gyro_x, dt);
        angle.angle_pitch = kalman_update(&kalman_pitch, angle_acc.angle_pitch, physical_data.gyro_y, dt);
        uart_printf(">Angle roll:%f,Angle pitch:%f\r\n", angle.angle_roll, angle.angle_pitch);
        /*uart_printf("Physical Accel: X=%f g, Y=%f g, Z=%f g | Physical Gyro: X=%f °/s, Y=%f °/s, Z=%f °/s\n",
                    physical_data.accel_x, physical_data.accel_y, physical_data.accel_z,
                    physical_data.gyro_x, physical_data.gyro_y, physical_data.gyro_z);*/
        // gpio_toggle(GPIOC, GPIO13); // Toggle an LED to indicate reading
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}
