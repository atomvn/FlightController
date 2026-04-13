#ifndef MPU6050_H
#define MPU6050_H
#include "driver/i2c.h"

/*********************** MPU6050 I2C Address ***********************/
#define MPU6050_ADDR       0x68

/*********************** MPU6050 Registers ***********************/
#define MPU6050_WHO_AM_I      0x75
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_GYRO_CFG      0x1B
#define MPU6050_ACCEL_CFG     0x1C
#define MPU6050_ACCEL_XH      0x3B
#define MPU6050_CONFIG        0x1A

/*********************** Manual Calibration Values, calculated by hand and experiment ***********************/
#define MANUAL_ACCELX_CALIB_VALUE 0.058f
#define MANUAL_ACCELY_CALIB_VALUE 0.015f
#define MANUAL_ACCELZ_CALIB_VALUE 0.153f

#define G           9.80665f
#define PI          3.141592653589793f
#define RAD_TO_DEG (180/PI)

/*********************** MPU6050 State ***********************/
typedef int32_t mpu6050_error_t;
#define MPU6050_OK             0
#define MPU6050_ERR_I2C       (-1)
#define MPU6050_MUTEX_ERROR   (-2)
#define MPU6050_INVALID_ARG   (-3)

/* MPU6050 raw data structure */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_raw_data_t;

/* MPU6050 physical data structure */
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_physical_data_t;

/* MPU6050 main state structure */
typedef struct {
    float angle_roll;
    float angle_pitch;
    mpu6050_raw_data_t raw_data;
    mpu6050_physical_data_t physical_data;
    SemaphoreHandle_t mutex;
    uint32_t timeout;
} mpu6050_t;

/* Global instance of MPU6050 state */
extern mpu6050_t g_mpu6050;

/* ======================== Public API ======================== */
mpu6050_error_t read_mpu6050_data(mpu6050_t *mpu);
void mpu6050_task(void *params);

#endif // MPU6050_H