#ifndef MPU6050_H
#define MPU6050_H
#include "driver/i2c.h"

/* I2C address */
#define MPU6050_ADDR       0x68

/* Registers */
#define MPU6050_WHO_AM_I      0x75
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_GYRO_CFG      0x1B
#define MPU6050_ACCEL_CFG     0x1C
#define MPU6050_ACCEL_XH      0x3B
#define MPU6050_CONFIG        0x1A

#define G 9.80665f
#define PI 3.141592653589793f
#define RAD_TO_DEG 57.2957795f

/* I2C*/
I2C_control i2c;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_raw_data;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_physical_data;

typedef struct {
    float gyro_x_offset;
    float gyro_y_offset;
    float gyro_z_offset;
} mpu6050_calibrating_offset;

typedef struct {
    float angle_roll;
    float angle_pitch;
} mpu6050_angle;

mpu6050_raw_data data;
mpu6050_physical_data physical_data;
mpu6050_angle angle_acc;
mpu6050_angle angle;

void mpu6050_write_to_reg(uint8_t reg, uint8_t data);
void mpu6050_init(void);
void mpu6050_read_accel_gyro(mpu6050_raw_data *data);
void mpu6050_task(void *params);
void convert_raw_to_physical(mpu6050_raw_data *raw, mpu6050_physical_data *data);
void calculate_mpu6050_angle_acc(mpu6050_angle* angle);
void calculate_calibrate_offset(mpu6050_calibrating_offset *offsets);
void apply_calibration(mpu6050_physical_data *data, mpu6050_calibrating_offset *offsets);

#endif