/**
 * @file i2c.c
 * @brief STM32F1 I2C Driver Implementation
 *
 * @details
 *
 * @author Hao Nguyen
 * @version 1.0 
 * @date 2026
 */

#include "FreeRTOS.h"
#include "task.h"
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/i2c.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>

#include "i2c.h"

/* ========================================================================
 * PRE-DEFINED CONFIGURATIONS
 * ======================================================================== */

/**
 * @brief Standard 100 kHz I2C configuration for 36 MHz APB clock
 */
const i2c_config_t I2C_CONFIG_100KHZ = {
    .speed_khz = 100,
    .apb_clock_mhz = 36
};

/**
 * @brief Fast 400 kHz I2C configuration for 36 MHz APB clock
 */
const i2c_config_t I2C_CONFIG_400KHZ = {
    .speed_khz = 400,
    .apb_clock_mhz = 36
};

/* ========================================================================
 * ERROR MESSAGE LOOKUP TABLE
 * ======================================================================== */

/**
 * @brief Human-readable error messages for I2C operations
 * @note Must match error code definitions in i2c_refactored.h
 */
static const char* i2c_error_messages[] = {
    "I2C: Operation successful",                    // I2C_OK (0)
    "I2C: Initialization failed",                   // I2C_ERR_INIT_FAILED (-1)
    "I2C: Operation timeout",                       // I2C_ERR_TIMEOUT (-2)
    "I2C: Slave NACK (device not responding)",      // I2C_ERR_NACK (-3)
    "I2C: Hardware error (BERR/ARLO)",              // I2C_ERR_HARDWARE (-4)
    "I2C: Bus stuck busy",                          // I2C_ERR_BUS_BUSY (-5)
};

#define I2C_ERROR_MSG_COUNT (sizeof(i2c_error_messages) / sizeof(i2c_error_messages[0]))

/* ========================================================================
 * HELPER FUNCTIONS
 * ======================================================================== */

/**
 * @brief Calculate time difference between two FreeRTOS ticks
 * @param early Earlier tick value
 * @param later Later tick value
 * @return Time difference in ticks (handles wrap-around)
 */
static TickType_t i2c_diff_ticks(TickType_t early, TickType_t later)
{
    if (later >= early)
        return later - early;
    return ~(TickType_t)0 - early + 1 + later;
}

/**
 * @brief Get current system tick count
 * @return Current tick count from FreeRTOS
 */
static inline TickType_t i2c_systicks(void)
{
    return xTaskGetTickCount();
}

/* ========================================================================
 * I2C RESET AND CONFIGURATION
 * ======================================================================== */

/**
 * @brief Reset I2C peripheral to clear any pending conditions
 * @param i2c I2C peripheral base address (I2C1, I2C2, or I2C3)
 * @return I2C_OK on success, I2C_ERR_HARDWARE if invalid peripheral
 *
 * @details
 * - Applies reset pulse via RCC peripheral reset register
 * - Clears all pending conditions and error flags
 * - Used during initialization and error recovery
 */
static i2c_status_t i2c_reset(uint32_t i2c)
{
    switch (i2c) {
    case I2C1:
        rcc_periph_reset_pulse(RST_I2C1);
        return I2C_OK;
    case I2C2:
        rcc_periph_reset_pulse(RST_I2C2);
        return I2C_OK;
#if defined(I2C3_BASE)
    case I2C3:
        rcc_periph_reset_pulse(RST_I2C3);
        return I2C_OK;
#endif
    default:
        return I2C_ERR_HARDWARE;
    }
}

/**
 * @brief Configure I2C clock timing parameters
 * @param i2c I2C peripheral base address
 * @param config Configuration structure with speed and APB clock
 * @return I2C_OK on success, I2C_ERR_INIT_FAILED on invalid speed
 *
 * @details
 * Timing Parameters Calculation:
 * - CCR (Clock Control Register) = APB_CLK / (2 * I2C_SPEED)
 * - TRISE = APB_CLK * (max_rise_time / 1000ns) + 1
 *
 * Examples (APB1 = 36 MHz):
 * - 100 kHz: CCR = 36MHz / 200kHz = 180, TRISE = 37
 * - 400 kHz: CCR = 36MHz / 800kHz = 45,  TRISE = 11
 *
 * Examples (APB1 = 72 MHz):
 * - 100 kHz: CCR = 72MHz / 200kHz = 360, TRISE = 73
 * - 400 kHz: CCR = 72MHz / 800kHz = 90,  TRISE = 22
 */
static i2c_status_t i2c_config_timing(uint32_t i2c, const i2c_config_t *config)
{
    if (!config)
        return I2C_ERR_INIT_FAILED;

    uint32_t apb_clock_hz = config->apb_clock_mhz * 1000000U;
    uint32_t i2c_speed_hz = config->speed_khz * 1000U;

    /* Validate speed range */
    if (config->speed_khz < 10 || config->speed_khz > 1000)
        return I2C_ERR_INIT_FAILED;

    /* Calculate CCR (Clock Control Register) */
    uint16_t ccr = apb_clock_hz / (2 * i2c_speed_hz);
    if (ccr < 4)
        ccr = 4;  /* Minimum value to avoid bus conflicts */

    /* Calculate TRISE: max rise time is 1000 ns */
    uint16_t trise = (apb_clock_hz * 1000) / 1000000000U + 1;

    /* Set clock frequency (in MHz) for status register timing */
    i2c_set_clock_frequency(i2c, config->apb_clock_mhz);

    /* Set rise time for open-drain pins */
    i2c_set_trise(i2c, trise);

    /* Set SCL frequency via CCR */
    i2c_set_dutycycle(i2c, I2C_CCR_DUTY_DIV2);  /* 50% duty cycle */
    i2c_set_ccr(i2c, ccr);

    return I2C_OK;
}

/* ========================================================================
 * PUBLIC API: INITIALIZATION
 * ======================================================================== */

/**
 * @brief Initialize I2C peripheral with specified configuration
 *
 * @param dev Pointer to I2C control structure (must not be NULL)
 * @param i2c I2C peripheral base address (I2C1, I2C2, or I2C3)
 * @param config Configuration structure (must not be NULL)
 *
 * @return I2C_OK on success
 * @return I2C_ERR_INIT_FAILED if dev==NULL, config==NULL, or invalid speed
 * @return I2C_ERR_HARDWARE if invalid I2C peripheral
 *
 * @pre dev and config must point to valid memory
 * @post I2C peripheral is enabled and ready for transactions
 *
 * @note
 * - Sets 7-bit addressing mode
 * - Configures own slave address to 0x00 (not using slave mode)
 * - Initializes error tracking and timeout values
 * - Clears any pending I2C conditions from previous usage
 *
 * @example
 * @code
 * I2C_control i2c_dev;
 * i2c_status_t err = i2c_init(&i2c_dev, I2C1, &I2C_CONFIG_100KHZ);
 * if (err != I2C_OK) {
 *     uart_printf("[ERROR] I2C init failed: %s\n", i2c_error_string(err));
 * }
 * @endcode
 */
i2c_status_t i2c_init(I2C_control *dev, uint32_t i2c, const i2c_config_t *config)
{
    /* Validate parameters */
    if (!dev || !config)
        return I2C_ERR_INIT_FAILED;

    /* Store configuration in control structure */
    dev->device = i2c;
    dev->speed_khz = config->speed_khz;
    dev->error_count = 0;
    dev->last_error = I2C_OK;

    /* Initialize timeout to 100 ms */
    dev->timeout = pdMS_TO_TICKS(100);

    /* Initialize semaphore handles to NULL (for future interrupt support) */
    dev->btf_event = NULL;
    dev->addr_event = NULL;
    dev->rxne_event = NULL;

    /* Disable peripheral during configuration */
    i2c_peripheral_disable(dev->device);

    /* Reset peripheral to clear any pending conditions */
    i2c_status_t err = i2c_reset(dev->device);
    if (err != I2C_OK) {
        dev->last_error = err;
        dev->error_count++;
        return err;
    }

    /* Clear stop bit to avoid generating spurious stop condition */
    I2C_CR1(dev->device) &= ~I2C_CR1_STOP;

    /* Configure timing based on selected speed */
    err = i2c_config_timing(dev->device, config);
    if (err != I2C_OK) {
        dev->last_error = err;
        dev->error_count++;
        return err;
    }

    /* Set addressing mode to 7-bit */
    i2c_set_standard_mode(dev->device);

    /* Set own slave address to 0x00 (not using slave mode) */
    i2c_set_own_7bit_slave_address(dev->device, 0x00);

    /* Enable peripheral after configuration complete */
    i2c_peripheral_enable(dev->device);

    return I2C_OK;
}

/* ========================================================================
 * PUBLIC API: ERROR HANDLING & DIAGNOSTICS
 * ======================================================================== */

/**
 * @brief Get human-readable error description
 *
 * @param error_code I2C error code
 * @param buf_out Buffer to store error message
 * @param buf_size Size of output buffer
 *
 * @return I2C_OK if message was generated successfully
 * @return I2C_ERR_INIT_FAILED if buf_out is NULL or buf_size is 0
 *
 * @pre buf_out must point to valid memory
 * @pre buf_size > 0
 * @post buf_out contains null-terminated error message
 *
 * @example
 * @code
 * char errmsg[64];
 * i2c_error_describe(i2c_dev.last_error, errmsg, sizeof(errmsg));
 * uart_printf("Error: %s\n", errmsg);
 * @endcode
 */
i2c_status_t i2c_error_describe(i2c_status_t error_code, char *buf_out, size_t buf_size)
{
    if (!buf_out || buf_size == 0)
        return I2C_ERR_INIT_FAILED;

    const char *msg = NULL;

    /* Find appropriate error message */
    if (error_code == I2C_OK) {
        msg = i2c_error_messages[0];
    } else if (error_code <= -1 && error_code >= -(I2C_ERROR_MSG_COUNT - 1)) {
        msg = i2c_error_messages[-error_code];
    } else {
        msg = "I2C: Unknown error code";
    }

    /* Copy message to output buffer, ensuring null termination */
    strncpy(buf_out, msg, buf_size - 1);
    buf_out[buf_size - 1] = '\0';

    return I2C_OK;
}

/**
 * @brief Get string name of I2C status code
 *
 * @param error_code I2C status code
 * @return Pointer to static status name string
 *
 * @note Useful for logging and debugging
 * @note Returned pointer is valid for program lifetime
 *
 * @example
 * @code
 * uart_printf("I2C Status: %s\n", i2c_error_string(I2C_ERR_NACK));
 * @endcode
 */
const char* i2c_error_string(i2c_status_t error_code)
{
    switch (error_code) {
    case I2C_OK:                return "I2C_OK";
    case I2C_ERR_INIT_FAILED:   return "I2C_ERR_INIT_FAILED";
    case I2C_ERR_TIMEOUT:       return "I2C_ERR_TIMEOUT";
    case I2C_ERR_NACK:          return "I2C_ERR_NACK";
    case I2C_ERR_HARDWARE:      return "I2C_ERR_HARDWARE";
    case I2C_ERR_BUS_BUSY:      return "I2C_ERR_BUS_BUSY";
    default:                    return "I2C_ERR_UNKNOWN";
    }
}

/**
 * @brief Record an error in the device control structure
 *
 * @param dev Pointer to I2C control structure
 * @param error_code Error code to record
 *
 * @note This is an internal helper; users typically just return error codes
 */
static inline void i2c_record_error(I2C_control *dev, i2c_status_t error_code)
{
    if (dev) {
        dev->last_error = error_code;
        if (error_code != I2C_OK)
            dev->error_count++;
    }
}

/* ========================================================================
 * PUBLIC API: BUS OPERATIONS
 * ======================================================================== */

/**
 * @brief Wait for I2C bus to become idle (not busy flag clear)
 *
 * @param dev Pointer to I2C control structure (must not be NULL)
 * @return I2C_OK if bus ready
 * @return I2C_ERR_BUS_BUSY if timeout waiting for bus ready
 * @return I2C_ERR_INIT_FAILED if dev is NULL
 *
 * @pre I2C peripheral must be initialized
 * @post Bus is ready for new start condition
 *
 * @details
 * - Checks I2C_SR2_BUSY flag
 * - Yields to other FreeRTOS tasks while waiting
 * - Returns error if timeout exceeded
 */
i2c_status_t i2c_busy_wait(I2C_control *dev)
{
    if (!dev)
        return I2C_ERR_INIT_FAILED;

    TickType_t start_time = i2c_systicks();

    while (I2C_SR2(dev->device) & I2C_SR2_BUSY) {
        if (i2c_diff_ticks(start_time, i2c_systicks()) > dev->timeout) {
            i2c_record_error(dev, I2C_ERR_BUS_BUSY);
            return I2C_ERR_BUS_BUSY;
        }
        taskYIELD();
    }

    return I2C_OK;
}

/**
 * @brief Recover I2C bus from stuck condition (I2C Spec 3.1.16)
 *
 * @param dev Pointer to I2C control structure (must not be NULL)
 * @return I2C_OK on success or attempted recovery
 * @return I2C_ERR_INIT_FAILED if dev is NULL
 *
 * @details
 * Bus recovery procedure:
 * 1. Generate stop condition to release bus
 * 2. Disable then re-enable I2C peripheral
 * 3. Clear error flags
 * 4. Wait for bus to become free
 *
 * Use this when:
 * - I2C bus appears stuck (SDA or SCL low)
 * - Previous transaction ended in error state
 * - Slave device is in an unknown state
 *
 * @note Full I2C spec recovery (9 SCL pulses) requires GPIO bit-banging.
 *       This simplified version resets the peripheral.
 */
i2c_status_t i2c_bus_recovery(I2C_control *dev)
{
    if (!dev)
        return I2C_ERR_INIT_FAILED;

    /* Generate stop condition to release bus */
    i2c_send_stop(dev->device);

    /* Disable and re-enable peripheral */
    i2c_peripheral_disable(dev->device);
    taskYIELD();  /* Let stop condition complete */
    i2c_peripheral_enable(dev->device);

    /* Clear any error flags */
    I2C_SR1(dev->device) = 0;
    I2C_SR2(dev->device) = 0;

    /* Wait for bus to become free */
    return i2c_busy_wait(dev);
}

/* ========================================================================
 * PUBLIC API: I2C TRANSACTIONS
 * ======================================================================== */

/**
 * @brief Start I2C transaction with device at specified address
 *
 * @param dev Pointer to I2C control structure (must not be NULL)
 * @param addr 7-bit slave address (0x00-0x7F)
 * @param rw I2C_READ for read transaction, I2C_WRITE for write transaction
 *
 * @return I2C_OK on success
 * @return I2C_ERR_INIT_FAILED if dev is NULL or addr invalid
 * @return I2C_ERR_BUS_BUSY if bus cannot be acquired
 * @return I2C_ERR_TIMEOUT if start condition timeout
 * @return I2C_ERR_NACK if slave does not acknowledge address
 *
 * @pre I2C peripheral must be initialized
 * @post I2C transaction has started; ready for read/write operations
 *
 * @details
 * Sequence:
 * 1. Wait for bus to become free (not busy)
 * 2. Clear address NACK flag
 * 3. Generate START condition
 * 4. Wait for START bit (SB flag)
 * 5. Send 7-bit address with R/W bit
 * 6. Wait for ADDR flag (address transmission complete)
 * 7. Handle address NACK if slave not present
 *
 * @example
 * @code
 * i2c_status_t err = i2c_start_addr(&i2c_dev, 0x68, I2C_WRITE);
 * if (err != I2C_OK) {
 *     uart_printf("[ERROR] Failed to start I2C: %s\n", i2c_error_string(err));
 * }
 * @endcode
 */
i2c_status_t i2c_start_addr(I2C_control *dev, uint8_t addr, uint8_t rw)
{
    if (!dev || addr > 0x7F)
        return I2C_ERR_INIT_FAILED;

    TickType_t start_time;

    /* Step 1: Wait until I2C is not busy before starting transaction */
    i2c_status_t err = i2c_busy_wait(dev);
    if (err != I2C_OK)
        return err;

    /* Step 2: Clear address NACK flag before starting transaction */
    I2C_SR1(dev->device) &= ~I2C_SR1_AF;

    /* Step 3: Clear stop bit to avoid generating spurious stop condition */
    i2c_clear_stop(dev->device);

    /* Step 4: Enable ACK for read transactions */
    if (rw == I2C_READ) {
        i2c_enable_ack(dev->device);
    }

    /* Step 5: Generate START condition */
    i2c_send_start(dev->device);

    /* Step 6: Wait for START bit (SB flag set) */
    start_time = i2c_systicks();
    while (!(I2C_SR1(dev->device) & I2C_SR1_SB)) {
        if (i2c_diff_ticks(start_time, i2c_systicks()) > dev->timeout) {
            i2c_record_error(dev, I2C_ERR_TIMEOUT);
            return I2C_ERR_TIMEOUT;
        }
        taskYIELD();
    }

    /* Step 7: Send 7-bit address with R/W flag */
    i2c_send_7bit_address(dev->device, addr, rw == I2C_READ ? I2C_READ : I2C_WRITE);

    /* Step 8: Wait for ADDR flag (address transmission complete) or NAK */
    start_time = i2c_systicks();
    while (!(I2C_SR1(dev->device) & I2C_SR1_ADDR)) {
        /* Check for address NACK (slave not responding) */
        if (I2C_SR1(dev->device) & I2C_SR1_AF) {
            i2c_send_stop(dev->device);
            (void)I2C_SR1(dev->device);  /* Clear AF flag */
            (void)I2C_SR2(dev->device);  /* Clear ADDR flag if any */
            i2c_record_error(dev, I2C_ERR_NACK);
            return I2C_ERR_NACK;
        }

        if (i2c_diff_ticks(start_time, i2c_systicks()) > dev->timeout) {
            i2c_send_stop(dev->device);
            i2c_record_error(dev, I2C_ERR_TIMEOUT);
            return I2C_ERR_TIMEOUT;
        }
        taskYIELD();
    }

    /* Step 9: Clear ADDR flag by reading SR2 after SR1 indicates address sent */
    (void)I2C_SR2(dev->device);

    return I2C_OK;
}

/**
 * @brief Send one byte on I2C bus
 *
 * @param dev Pointer to I2C control structure (must not be NULL)
 * @param byte Byte value to transmit (0x00-0xFF)
 *
 * @return I2C_OK on successful transmission
 * @return I2C_ERR_INIT_FAILED if dev is NULL
 * @return I2C_ERR_TIMEOUT if transmission timeout
 * @return I2C_ERR_NACK if slave NACK (typically means end of data read)
 * @return I2C_ERR_HARDWARE if hardware error detected
 *
 * @pre I2C transaction must be started with i2c_start_addr()
 * @pre Peripheral must be in master transmitter mode (address sent with write)
 *
 * @details
 * - Places byte in I2C_DR (data register)
 * - Waits for byte transmission complete (BTF flag)
 * - Yields to other tasks while waiting
 * - Returns error if timeout exceeded
 *
 * @example
 * @code
 * err = i2c_write(&i2c_dev, 0xA5);  // Send 0xA5
 * if (err != I2C_OK)
 *     uart_printf("[ERROR] Write failed: %s\n", i2c_error_string(err));
 * @endcode
 */
i2c_status_t i2c_write(I2C_control *dev, uint8_t byte)
{
    if (!dev)
        return I2C_ERR_INIT_FAILED;

    TickType_t start_time = i2c_systicks();

    /* Send byte */
    i2c_send_data(dev->device, byte);

    /* Wait for byte transmission complete (BTF = Byte Transfer Finished) */
    while (!(I2C_SR1(dev->device) & I2C_SR1_BTF)) {
        /* Check for NACK from slave */
        if (I2C_SR1(dev->device) & I2C_SR1_AF) {
            I2C_SR1(dev->device) &= ~I2C_SR1_AF;  /* Clear AF flag */
            i2c_record_error(dev, I2C_ERR_NACK);
            return I2C_ERR_NACK;
        }

        /* Check for hardware errors (BERR=Bus Error, ARLO=Arbitration Lost) */
        if (I2C_SR1(dev->device) & (I2C_SR1_BERR | I2C_SR1_ARLO)) {
            I2C_SR1(dev->device) &= ~(I2C_SR1_BERR | I2C_SR1_ARLO);
            i2c_record_error(dev, I2C_ERR_HARDWARE);
            return I2C_ERR_HARDWARE;
        }

        if (i2c_diff_ticks(start_time, i2c_systicks()) > dev->timeout) {
            i2c_record_error(dev, I2C_ERR_TIMEOUT);
            return I2C_ERR_TIMEOUT;
        }
        taskYIELD();
    }

    return I2C_OK;
}

/**
 * @brief Read one byte from I2C bus
 *
 * @param dev Pointer to I2C control structure (must not be NULL)
 * @param last_byte true if this is the last byte to read, false otherwise
 * @param byte_out Pointer to store received byte (must not be NULL)
 *
 * @return I2C_OK on successful reception
 * @return I2C_ERR_INIT_FAILED if dev or byte_out is NULL
 * @return I2C_ERR_TIMEOUT if reception timeout
 * @return I2C_ERR_HARDWARE if hardware error detected
 *
 * @pre I2C transaction must be started with i2c_start_addr() using I2C_READ
 * @pre Peripheral must be in master receiver mode (address sent with read)
 * @post Received byte available in *byte_out
 *
 * @details
 * - For multi-byte reads: set last_byte=false for all but final byte
 * - For final byte: set last_byte=true to generate STOP condition
 * - Waits for byte reception complete (RXNE flag)
 * - Yields to other tasks while waiting
 * - Returns error if timeout exceeded
 *
 * @example
 * @code
 * uint8_t data[2];
 * err = i2c_read(&i2c_dev, false, &data[0]);  // Read first byte
 * if (err != I2C_OK) return err;
 *
 * err = i2c_read(&i2c_dev, true, &data[1]);   // Read last byte (stop after)
 * if (err != I2C_OK) return err;
 * @endcode
 */
i2c_status_t i2c_read(I2C_control *dev, bool last_byte, uint8_t *byte_out)
{
    if (!dev || !byte_out)
        return I2C_ERR_INIT_FAILED;

    TickType_t start_time = i2c_systicks();

    /* If this is the last byte, disable ACK to signal end of read */
    if (last_byte) {
        i2c_disable_ack(dev->device);
    }

    /* Wait for byte to be received (RXNE = RxNE flag set) */
    while (!(I2C_SR1(dev->device) & I2C_SR1_RxNE)) {
        /* Check for hardware errors */
        if (I2C_SR1(dev->device) & (I2C_SR1_BERR | I2C_SR1_ARLO)) {
            I2C_SR1(dev->device) &= ~(I2C_SR1_BERR | I2C_SR1_ARLO);
            i2c_record_error(dev, I2C_ERR_HARDWARE);
            return I2C_ERR_HARDWARE;
        }

        if (i2c_diff_ticks(start_time, i2c_systicks()) > dev->timeout) {
            i2c_record_error(dev, I2C_ERR_TIMEOUT);
            return I2C_ERR_TIMEOUT;
        }
        taskYIELD();
    }

    /* Read byte from data register */
    *byte_out = i2c_get_data(dev->device);

    return I2C_OK;
}

/**
 * @brief Write one byte and immediately restart for read (repeated START)
 *
 * @param dev Pointer to I2C control structure (must not be NULL)
 * @param byte Byte to transmit before restart
 * @param new_addr 7-bit slave address for read phase
 *
 * @return I2C_OK on successful completion
 * @return I2C_ERR_INIT_FAILED if dev is NULL or address invalid
 * @return I2C_ERR_TIMEOUT if transmission or restart timeout
 * @return I2C_ERR_NACK if slave does not acknowledge address or write
 * @return I2C_ERR_HARDWARE if hardware error detected
 *
 * @pre I2C transaction must be in master transmitter mode
 * @post I2C bus is ready for data reception from new_addr
 *
 * @details
 * Used for I2C transactions requiring write followed by read:
 * 1. Write byte to slave (typically register address or command)
 * 2. Generate repeated START condition (without STOP in between)
 * 3. Send address with read bit
 * 4. Bus is now ready for i2c_read() calls
 *
 * Typical use case: Read register from I2C slave
 * - Write register address
 * - Repeated START
 * - Read register contents
 *
 * @example
 * @code
 * // Read MPU6050 accelerometer X register (0x3B)
 * i2c_status_t err = i2c_start_addr(&i2c_dev, 0x68, I2C_WRITE);
 * if (err != I2C_OK) return err;
 *
 * err = i2c_write(&i2c_dev, 0x3B);  // Reg address
 * if (err != I2C_OK) return err;
 *
 * err = i2c_write_restart(&i2c_dev, 0x00, 0x68);  // Repeated START for read
 * if (err != I2C_OK) return err;
 *
 * uint8_t accel_x;
 * err = i2c_read(&i2c_dev, true, &accel_x);  // Read final byte
 * if (err != I2C_OK) return err;
 * @endcode
 */
i2c_status_t i2c_write_restart(I2C_control *dev, uint8_t byte, uint8_t new_addr)
{
    if (!dev || new_addr > 0x7F)
        return I2C_ERR_INIT_FAILED;

    TickType_t start_time;

    /* Critical section: coordinate byte write and repeated START */
    taskENTER_CRITICAL();
    {
        i2c_send_data(dev->device, byte);
        /* Must set START before byte transmits to create repeated START */
        i2c_send_start(dev->device);
    }
    taskEXIT_CRITICAL();

    /* Wait for byte transmission to complete */
    start_time = i2c_systicks();
    while (!(I2C_SR1(dev->device) & I2C_SR1_BTF)) {
        /* Check for hardware errors */
        if (I2C_SR1(dev->device) & (I2C_SR1_BERR | I2C_SR1_ARLO)) {
            I2C_SR1(dev->device) &= ~(I2C_SR1_BERR | I2C_SR1_ARLO);
            i2c_record_error(dev, I2C_ERR_HARDWARE);
            return I2C_ERR_HARDWARE;
        }

        if (i2c_diff_ticks(start_time, i2c_systicks()) > dev->timeout) {
            i2c_record_error(dev, I2C_ERR_TIMEOUT);
            return I2C_ERR_TIMEOUT;
        }
        taskYIELD();
    }

    /* Wait for repeated START condition to complete (SB flag set) */
    start_time = i2c_systicks();
    while (!((I2C_SR1(dev->device) & I2C_SR1_SB) &&
             (I2C_SR2(dev->device) & (I2C_SR2_MSL | I2C_SR2_BUSY)))) {
        if (i2c_diff_ticks(start_time, i2c_systicks()) > dev->timeout) {
            i2c_record_error(dev, I2C_ERR_TIMEOUT);
            return I2C_ERR_TIMEOUT;
        }
        taskYIELD();
    }

    /* Send address with READ bit */
    i2c_send_7bit_address(dev->device, new_addr, I2C_READ);

    /* Wait for ADDR flag or NACK */
    start_time = i2c_systicks();
    while (!(I2C_SR1(dev->device) & I2C_SR1_ADDR)) {
        /* Check for address NACK */
        if (I2C_SR1(dev->device) & I2C_SR1_AF) {
            i2c_send_stop(dev->device);
            (void)I2C_SR1(dev->device);  /* Clear AF flag */
            (void)I2C_SR2(dev->device);  /* Clear ADDR flag if any */
            i2c_record_error(dev, I2C_ERR_NACK);
            return I2C_ERR_NACK;
        }

        /* Check for hardware errors */
        if (I2C_SR1(dev->device) & (I2C_SR1_BERR | I2C_SR1_ARLO)) {
            I2C_SR1(dev->device) &= ~(I2C_SR1_BERR | I2C_SR1_ARLO);
            i2c_send_stop(dev->device);
            i2c_record_error(dev, I2C_ERR_HARDWARE);
            return I2C_ERR_HARDWARE;
        }

        if (i2c_diff_ticks(start_time, i2c_systicks()) > dev->timeout) {
            i2c_send_stop(dev->device);
            i2c_record_error(dev, I2C_ERR_TIMEOUT);
            return I2C_ERR_TIMEOUT;
        }
        taskYIELD();
    }

    /* Clear ADDR flag by reading SR2 */
    (void)I2C_SR2(dev->device);

    return I2C_OK;
}

/**
 * @brief Stop I2C transaction and release bus
 *
 * @param dev Pointer to I2C control structure (must not be NULL)
 * @return I2C_OK on success
 * @return I2C_ERR_INIT_FAILED if dev is NULL
 * @return I2C_ERR_BUS_BUSY if bus remains busy after STOP
 *
 * @pre I2C transaction must be active (started with i2c_start_addr)
 * @post I2C bus is released and available for other transactions
 *
 * @details
 * - Generates STOP condition
 * - Waits for BUSY flag to clear
 * - Should be called after final read() or write()
 *
 * @note
 * Some operations (like final i2c_read() with last_byte=true) may generate
 * STOP condition automatically. Calling this after such operations is safe
 * but may be unnecessary.
 *
 * @example
 * @code
 * err = i2c_start_addr(&i2c_dev, 0x68, I2C_WRITE);
 * if (err != I2C_OK) goto error;
 *
 * err = i2c_write(&i2c_dev, 0x3B);
 * if (err != I2C_OK) goto error;
 *
 * err = i2c_stop(&i2c_dev);  // Release bus
 * if (err != I2C_OK) goto error;
 *
 * return I2C_OK;
 * error:
 *     return err;
 * @endcode
 */
i2c_status_t i2c_stop(I2C_control *dev)
{
    if (!dev)
        return I2C_ERR_INIT_FAILED;

    i2c_send_stop(dev->device);
    return i2c_busy_wait(dev);
}
