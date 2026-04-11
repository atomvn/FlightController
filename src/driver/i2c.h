#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "libopencm3/cm3/common.h"
#include <libopencm3/stm32/f1/memorymap.h>
#include "libopencm3/stm32/f1/i2c.h"

/* ======================== Error Codes ======================== */

/**
 * @brief I2C status/error codes (return type for all functions)
 * 
 * @note Negative values indicate errors, zero indicates success
 */
typedef int32_t i2c_status_t;

#define I2C_OK                0      ///< Success
#define I2C_ERR_INIT_FAILED   (-1)   ///< Initialization failed
#define I2C_ERR_TIMEOUT       (-2)   ///< Operation timeout
#define I2C_ERR_NACK          (-3)   ///< Slave NACK
#define I2C_ERR_HARDWARE      (-4)   ///< Hardware error (BERR/ARLO)
#define I2C_ERR_BUS_BUSY      (-5)   ///< Bus stuck busy

/* ======================== Configuration ======================== */

/**
 * @brief I2C bus configuration structure
 * 
 * Defines the speed and clock parameters for I2C initialization.
 * Pre-defined configurations available: I2C_CONFIG_100KHZ, I2C_CONFIG_400KHZ
 */
typedef struct {
    uint16_t speed_khz;         ///< Desired I2C speed in kHz (100, 400, 1000)
    uint16_t apb_clock_mhz;     ///< APB peripheral clock in MHz (36, 48, 72)
} i2c_config_t;

extern const i2c_config_t I2C_CONFIG_100KHZ;   ///< 100 kHz standard mode
extern const i2c_config_t I2C_CONFIG_400KHZ;   ///< 400 kHz fast mode

/* ======================== Control Structure ======================== */

/**
 * @brief I2C bus control structure for driver state management
 * 
 * Each I2C bus instance requires its own I2C_control structure.
 * Must be initialized with i2c_init() before use.
 * 
 * @note The timeout field MUST be initialized (set in i2c_init).
 *       Do not rely on uninitialized value.
 */
typedef struct {
    uint32_t device;                    ///< libopencm3 I2C peripheral ID
    uint32_t timeout;                   ///< Timeout in FreeRTOS ticks (MUST be initialized)
    uint16_t speed_khz;                 ///< Configured I2C speed in kHz
    SemaphoreHandle_t btf_event;        ///< (Reserved for interrupt support)
    SemaphoreHandle_t addr_event;       ///< (Reserved for interrupt support)
    SemaphoreHandle_t rxne_event;       ///< (Reserved for interrupt support)
    uint32_t error_count;               ///< Cumulative error counter (diagnostic)
    i2c_status_t last_error;            ///< Most recent error code (diagnostic)
} I2C_control;

/* ======================== Public API ======================== */

/**
 * @brief Initialize I2C bus at specified speed
 * 
 * Configures the I2C peripheral, GPIO pins, and control structure.
 * The timeout field is automatically initialized to 100 ms.
 * 
 * @param dev I2C control structure (must be non-NULL)
 * @param i2c libopencm3 I2C peripheral (I2C1, I2C2, I2C3)
 * @param config I2C configuration with speed and clock parameters
 * 
 * @return I2C_OK on success, error code on failure
 * 
 * @pre dev is non-NULL
 * @pre config is non-NULL
 * @pre i2c is a valid I2C peripheral (I2C1, I2C2, etc.)
 * @post dev->timeout = pdMS_TO_TICKS(100)
 * @post dev->error_count = 0
 * 
 * @example
 * I2C_control i2c_bus;
 * i2c_status_t err = i2c_init(&i2c_bus, I2C1, &I2C_CONFIG_100KHZ);
 * if (err != I2C_OK) {
 *     uart_printf("I2C init failed: %s\n", i2c_error_string(err));
 * }
 */
i2c_status_t i2c_init(I2C_control *dev, uint32_t i2c, const i2c_config_t *config);

/**
 * @brief Wait for I2C bus to become idle
 * 
 * Polls the bus busy flag (SR2.BUSY) until the bus is free or timeout occurs.
 * Yields to other FreeRTOS tasks while waiting.
 * 
 * @param dev I2C control structure
 * 
 * @return I2C_OK if bus is idle, error code on timeout/failure
 * 
 * @pre dev is initialized with i2c_init()
 * @post No other I2C master is using the bus
 */
i2c_status_t i2c_busy_wait(I2C_control *dev);

/**
 * @brief Recover I2C bus from stuck condition (I2C Spec 3.1.16)
 * 
 * Implements the I2C specification bus recovery procedure:
 *   1. Generate up to 9 SCL clock pulses
 *   2. Release SDA after SCL is released
 *   3. Generate STOP condition
 * 
 * Note: Full implementation requires GPIO-based SCL/SDA control.
 * 
 * @param dev I2C control structure
 * 
 * @return I2C_OK on success, error code on failure
 * 
 * @pre dev is initialized
 * @post Bus is recovered or still stuck (caller should retry)
 */
i2c_status_t i2c_bus_recovery(I2C_control *dev);

/**
 * @brief Start I2C transaction with 7-bit address
 * 
 * Generates START condition and sends the slave address with R/W bit.
 * Waits for slave acknowledgment (ADDR flag) or NACK.
 * 
 * @param dev I2C control structure
 * @param addr Slave 7-bit address (0x00-0x7F)
 * @param rw I2C_READ or I2C_WRITE direction
 * 
 * @return I2C_OK if slave acknowledged, I2C_ERR_NACK if slave didn't acknowledge,
 *         other error codes on timeout/hardware error
 * 
 * @pre dev is initialized
 * @pre addr is valid 7-bit address (0x00-0x7F)
 * @post START condition has been sent
 * @post Slave has acknowledged (ADDR flag cleared)
 */
i2c_status_t i2c_start_addr(I2C_control *dev, uint8_t addr, uint8_t rw);

/**
 * @brief Write one byte to I2C bus
 * 
 * Sends a data byte and waits for byte transfer to complete (BTF flag).
 * Automatically handles slave acknowledgment or NACK.
 * 
 * @param dev I2C control structure
 * @param byte Data byte to transmit (0x00-0xFF)
 * 
 * @return I2C_OK on success, error code on hardware error/timeout
 * 
 * @pre i2c_start_addr() has been called
 * @post Byte has been transmitted and acknowledged
 */
i2c_status_t i2c_write(I2C_control *dev, uint8_t byte);

/**
 * @brief Read one byte from I2C bus
 * 
 * Receives a data byte from slave device.
 * If this is the last byte to read, sends NACK to signal end of transfer.
 * 
 * @param dev I2C control structure
 * @param last_byte true if this is the last/only byte to read
 * @param byte_out Pointer to store received byte
 * 
 * @return I2C_OK on success, error code on hardware error/timeout
 * 
 * @pre i2c_start_addr() has been called with I2C_READ
 * @pre byte_out is non-NULL
 * @post byte_out contains the received data
 * @post If last_byte=true, NACK has been sent to slave
 * 
 * @example
 * uint8_t data;
 * i2c_status_t err = i2c_read(&i2c_bus, true, &data);  // Last byte
 * if (err == I2C_OK) {
 *     uart_printf("Received: 0x%02x\n", data);
 * }
 */
i2c_status_t i2c_read(I2C_control *dev, bool last_byte, uint8_t *byte_out);

/**
 * @brief Write data byte and generate repeated START for read
 * 
 * Advanced I2C operation combining write + repeated START.
 * Atomically sends byte, generates repeated START, and addresses slave for read.
 * 
 * @param dev I2C control structure
 * @param byte Data byte to transmit
 * @param new_addr Slave address for read phase (7-bit)
 * 
 * @return I2C_OK on success, error code on failure
 * 
 * @pre i2c_start_addr() has been called with I2C_WRITE
 * @post Repeated START has been generated
 * @post Slave has acknowledged new address
 * 
 * @example
 * // I2C burst read: send register address, then read data
 * i2c_start_addr(&i2c_bus, 0x68, I2C_WRITE);
 * i2c_write(&i2c_bus, 0x3B);  // WHO_AM_I register
 * i2c_write_restart(&i2c_bus, 0xFF, 0x68);  // Repeated START to read
 * uint8_t id;
 * i2c_read(&i2c_bus, true, &id);
 * i2c_stop(&i2c_bus);
 */
i2c_status_t i2c_write_restart(I2C_control *dev, uint8_t byte, uint8_t new_addr);

/**
 * @brief Send STOP condition on I2C bus
 * 
 * Ends the current I2C transaction and releases the bus.
 * 
 * @param dev I2C control structure
 * 
 * @return I2C_OK on success, error code on failure
 * 
 * @pre i2c_start_addr() has been called
 * @post STOP condition has been generated
 * @post Bus is idle and available for other masters
 */
i2c_status_t i2c_stop(I2C_control *dev);

/* ======================== Error Handling ======================== */

/**
 * @brief Get human-readable error description
 * 
 * @param error_code I2C error code
 * @param buf_out Buffer to store error message
 * @param buf_size Size of output buffer
 * 
 * @return I2C_OK if message was generated successfully
 * 
 * @pre buf_out is non-NULL
 * @pre buf_size > 0
 * @post buf_out contains null-terminated error message
 * 
 * @example
 * char errmsg[64];
 * i2c_error_describe(i2c_dev.last_error, errmsg, sizeof(errmsg));
 * uart_printf("Error: %s\n", errmsg);
 */
i2c_status_t i2c_error_describe(i2c_status_t error_code, char *buf_out, size_t buf_size);

/**
 * @brief Get error string from error code (legacy function)
 * 
 * Returns a static string buffer with error description.
 * Less safe than i2c_error_describe() but simpler for printf-style usage.
 * 
 * @param error_code I2C error code
 * @return Pointer to static error message string
 * 
 * @example
 * uart_printf("Error: %s\n", i2c_error_string(err));
 */
const char* i2c_error_string(i2c_status_t error_code);

#endif  // I2C_H
