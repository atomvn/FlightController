#ifndef I2C_H
#define I2C_H
#include <stdint.h>
#include <stdbool.h>
#include <setjmp.h>
#include "libopencm3/cm3/common.h"
#include <libopencm3/stm32/f1/memorymap.h>
#include "libopencm3/stm32/f1/i2c.h"

typedef enum {
    I2C_ck = 0,
    I2C_addr_timeout,
    I2C_addr_nack,
    I2C_write_timeout,
    I2C_read_timeout
} I2C_errors;

typedef struct {
    uint32_t device;
    uint32_t timeout;
} I2C_control;

extern jmp_buf i2c_jmpbuf;

const char* i2c_error(I2C_errors error_code);
void i2c_init(I2C_control* dev, uint32_t i2c, uint32_t ticks);
void i2c_wait_busy(I2C_control* dev);
void i2c_start_addr(I2C_control* dev, uint8_t addr, int rw);
void i2c_write(I2C_control* dev, uint8_t data);
void i2c_write_restart(I2C_control* dev, uint8_t data, uint8_t addr);
uint8_t i2c_read(I2C_control* dev, bool lastflag);

inline void i2c_stop(I2C_control* dev) {
    i2c_send_stop(dev->device);
}

#endif