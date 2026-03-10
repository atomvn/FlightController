#include "FreeRTOS.h"
#include <task.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/i2c.h>
#include "libopencm3/stm32/f1/gpio.h"
#include "libopencm3/stm32/f1/usart.h"

#include "mpu6050.h"
#include "driver/uart.h"

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
    mpu6050_write_to_reg(MPU6050_GYRO_CFG, 0x10); 
    mpu6050_write_to_reg(MPU6050_ACCEL_CFG, 0x10); 
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

    data->accel_x = (raw[0] << 8) | raw[1];
    data->accel_y = (raw[2] << 8) | raw[3];
    data->accel_z = (raw[4] << 8) | raw[5];
    data->gyro_x  = (raw[8] << 8) | raw[9];
    data->gyro_y  = (raw[10] << 8) | raw[11];
    data->gyro_z  = (raw[12] << 8) | raw[13];
}

void convert_raw_to_physical(mpu6050_raw_data *raw, mpu6050_data *data) {
    data->accel_x = (raw->accel_x / 4096.0f) * G;
    data->accel_y = (raw->accel_y / 4096.0f) * G;
    data->accel_z = (raw->accel_z / 4096.0f) * G;
    data->gyro_x  = raw->gyro_x / 32.8f;
    data->gyro_y  = raw->gyro_y / 32.8f;
    data->gyro_z  = raw->gyro_z / 32.8f;
}

void calibrate_mpu6050(mpu6050_calibarting_offset *offsets) {
    const int samples = 300;
    mpu6050_calibarting_offset offset_sum = {0};
    mpu6050_raw_data data;
    mpu6050_data physical_data;

    for (int i = 0; i < samples; i++) {
        mpu6050_read_accel_gyro(&data);
        convert_raw_to_physical(&data, &physical_data);
        offset_sum.accel_x_offset += physical_data.accel_x;
        offset_sum.accel_y_offset += physical_data.accel_y;
        offset_sum.accel_z_offset += physical_data.accel_z;
        offset_sum.gyro_x_offset  += physical_data.gyro_x;
        offset_sum.gyro_y_offset  += physical_data.gyro_y;
        offset_sum.gyro_z_offset  += physical_data.gyro_z;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    offsets->accel_x_offset = offset_sum.accel_x_offset / samples;
    offsets->accel_y_offset = offset_sum.accel_y_offset / samples;
    offsets->accel_z_offset = offset_sum.accel_z_offset / samples;
    offsets->gyro_x_offset  = offset_sum.gyro_x_offset / samples;
    offsets->gyro_y_offset  = offset_sum.gyro_y_offset / samples;
    offsets->gyro_z_offset  = offset_sum.gyro_z_offset / samples;
}

void apply_calibration(mpu6050_data *data, mpu6050_calibarting_offset *offsets) {
    data->accel_x -= offsets->accel_x_offset;
    data->accel_y -= offsets->accel_y_offset;
    data->accel_z -= offsets->accel_z_offset;
    data->gyro_x  -= offsets->gyro_x_offset;
    data->gyro_y  -= offsets->gyro_y_offset;
    data->gyro_z  -= offsets->gyro_z_offset;
}

void mpu6050_task(void *params) {
    (void)params;
    mpu6050_raw_data data;
    mpu6050_data physical_data;
    mpu6050_calibarting_offset offsets;
    mpu6050_init();
    calibrate_mpu6050(&offsets);
    uart_send_string("[LOG] [mpu6050_task] Done MPU6050 init\n");
    while (1) {
        mpu6050_read_accel_gyro(&data);
        convert_raw_to_physical(&data, &physical_data);
        apply_calibration(&physical_data, &offsets);
        uart_printf("Physical Accel: X=%f m/s², Y=%f m/s², Z=%f m/s² | Physical Gyro: X=%f °/s, Y=%f °/s, Z=%f °/s\n",
                    physical_data.accel_x, physical_data.accel_y, physical_data.accel_z,
                    physical_data.gyro_x, physical_data.gyro_y, physical_data.gyro_z);
        gpio_toggle(GPIOC, GPIO13); // Toggle an LED to indicate reading
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
