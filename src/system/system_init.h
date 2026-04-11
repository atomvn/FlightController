#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================
 * STATUS & ERROR CODES
 * ======================================================================== */

/**
 * @brief System initialization status and error codes
 * 
 * Error codes are negative for failures, zero for success.
 * Can be used with system_error_string() to get human-readable messages.
 */
typedef int32_t system_status_t;

#define SYSTEM_OK                   0      ///< Success
#define SYSTEM_ERR_PLL_TIMEOUT      (-1)   ///< PLL failed to lock
#define SYSTEM_ERR_GPIO_CONFIG      (-2)   ///< GPIO configuration error
#define SYSTEM_ERR_USART_CONFIG     (-3)   ///< UART configuration error
#define SYSTEM_ERR_I2C_CONFIG       (-4)   ///< I2C configuration error
#define SYSTEM_ERR_PWM_CONFIG       (-5)   ///< PWM configuration error
#define SYSTEM_ERR_INIT_FAILED      (-6)   ///< Generic initialization failure
#define SYSTEM_ERR_INVALID_CONFIG   (-7)   ///< Invalid configuration parameters

/* ========================================================================
 * CONFIGURATION STRUCTURES
 * ======================================================================== */

/**
 * @brief Clock configuration parameters
 * 
 * Allows flexible clock setup for different board variants:
 * - 8 MHz HSE → 72 MHz (standard)
 * - 16 MHz HSE → 64 MHz (slower, for noisy boards)
 * - 25 MHz HSE → 50 MHz (low power variant)
 */
typedef struct {
    uint32_t hse_freq_mhz;          ///< External oscillator frequency (4-25 MHz)
    uint32_t pll_target_mhz;        ///< Desired PLL output (max 72 MHz for STM32F1)
    uint8_t apb1_prescaler;         ///< APB1 divider (1, 2, 4, 8, 16)
    uint8_t apb2_prescaler;         ///< APB2 divider (1, 2, 4, 8, 16)
} system_clock_config_t;

/**
 * @brief UART configuration parameters
 * 
 * Supports various baud rates and configurations:
 * - UART1: 38400 bps telemetry (TX-only or full-duplex)
 * - UART2: 100000 bps S-BUS receiver (RX-only, per protocol spec)
 */
typedef struct {
    uint32_t peripheral;            ///< USART1 or USART2
    uint32_t baud_rate;             ///< Baud rate (9600, 38400, 100000, etc.)
    uint8_t data_bits;              ///< Data bits (8 or 9)
    uint8_t stop_bits;              ///< Stop bits (1 or 2)
    uint8_t parity;                 ///< Parity (NONE, ODD, EVEN)
    bool tx_enabled;                ///< Enable transmitter
    bool rx_enabled;                ///< Enable receiver
} system_uart_config_t;

/**
 * @brief I2C configuration parameters
 * 
 * Configures I2C bus for sensor communication
 * - Standard mode: 100 kHz (safe, widely compatible)
 * - Fast mode: 400 kHz (faster, requires shorter cable)
 * - Fast mode+: 1000 kHz (risky, not recommended for flight control)
 */
typedef struct {
    uint32_t peripheral;            ///< I2C1, I2C2 (typically I2C1)
    uint16_t speed_khz;             ///< 100, 400, or 1000 kHz
    uint32_t timeout_ms;            ///< Transaction timeout
} system_i2c_config_t;

/**
 * @brief System diagnostics structure
 * 
 * Used to query system status after initialization
 */
typedef struct {
    uint32_t core_clock_hz;         ///< Actual core clock frequency (Hz)
    uint32_t apb1_clock_hz;         ///< APB1 peripheral clock (Hz)
    uint32_t apb2_clock_hz;         ///< APB2 peripheral clock (Hz)
    bool pll_locked;                ///< PLL lock status
    uint32_t uptime_ms;             ///< System uptime since boot
    struct {
        bool clock_ok;              ///< Clock initialization success
        bool gpio_ok;               ///< GPIO initialization success
        bool uart1_ok;              ///< UART1 initialization success
        bool uart2_ok;              ///< UART2 initialization success
        bool i2c_ok;                ///< I2C initialization success
        bool pwm_ok;                ///< PWM initialization success
    } peripherals;
} system_diagnostics_t;

/* ========================================================================
 * PUBLIC API
 * ======================================================================== */

/**
 * @brief Initialize STM32F1 system with default configuration
 * 
 * **Default Configuration:**
 * - Clock: 8 MHz HSE → 72 MHz PLL
 * - UART1: 38400 bps TX (PA9)
 * - UART2: 100000 bps RX (PA3, S-BUS)
 * - I2C1: 100 kHz (PB6/PB7)
 * - PWM: 50 Hz motor + 2 servos
 * - GPIO: LED (PC13), debug pins
 * - Watchdog: Enabled (500 ms timeout)
 * 
 * @return SYSTEM_OK on success
 * @return SYSTEM_ERR_PLL_TIMEOUT if PLL fails to lock
 * @return SYSTEM_ERR_GPIO_CONFIG if GPIO setup fails
 * @return Other error codes for specific peripheral failures
 * 
 * @pre Called from main() before FreeRTOS scheduler starts
 * @pre No interrupts should be running yet
 * @post All peripherals ready for use
 * @post LED blinking (if enabled via diagnostics)
 * @post UART1 ready for logging
 * @post UART2 ready to receive S-BUS frames
 * @post I2C ready for sensor communication
 * @post PWM ready for motor/servo control
 * 
 * @note Idempotent - safe to call multiple times (guarded by static flag)
 * @note If called more than once, returns SYSTEM_OK without re-initializing
 * 
 * @example
 * @code
 * system_status_t err = system_init();
 * if (err != SYSTEM_OK) {
 *     uart_printf("[FATAL] System init failed: %s\n", system_error_string(err));
 *     for (;;) __asm__("wfi");  // Halt
 * }
 * // All hardware is now ready - can start FreeRTOS
 * @endcode
 */
system_status_t system_init(void);

/**
 * @brief Initialize STM32F1 system with custom configuration
 * 
 * Allows fine-grained control over clock, UART, and I2C settings.
 * Useful for:
 * - Testing different configurations
 * - Supporting board variants
 * - Power-optimized modes
 * 
 * @param clock_cfg Clock configuration (if NULL, uses defaults)
 * @param uart1_cfg UART1 configuration (if NULL, uses defaults)
 * @param uart2_cfg UART2 configuration (if NULL, uses defaults)
 * @param i2c_cfg I2C configuration (if NULL, uses defaults)
 * 
 * @return SYSTEM_OK on success, error code on failure
 * 
 * @example
 * @code
 * system_clock_config_t custom_clock = {
 *     .hse_freq_mhz = 16,
 *     .pll_target_mhz = 64,
 *     .apb1_prescaler = 2,
 *     .apb2_prescaler = 1
 * };
 * 
 * system_status_t err = system_init_custom(&custom_clock, NULL, NULL, NULL);
 * if (err != SYSTEM_OK) {
 *     uart_printf("[ERROR] Custom init failed\n");
 *     return;
 * }
 * @endcode
 */
system_status_t system_init_custom(
    const system_clock_config_t *clock_cfg,
    const system_uart_config_t *uart1_cfg,
    const system_uart_config_t *uart2_cfg,
    const system_i2c_config_t *i2c_cfg
);

/**
 * @brief Reconfigure UART at runtime
 * 
 * Changes UART baud rate or mode without reinitializing GPIO.
 * Safe to call after system_init() if UART needs reconfiguration.
 * 
 * @param usart USART peripheral (USART1 or USART2)
 * @param baud_rate New baud rate (bps)
 * 
 * @return SYSTEM_OK on success, error on invalid peripheral
 * 
 * @note Disables UART during reconfiguration (brief interruption)
 * @note Existing data in TX/RX buffers is lost
 */
system_status_t system_reconfigure_uart(uint32_t usart, uint32_t baud_rate);

/**
 * @brief Get current system diagnostics and status
 * 
 * Returns detailed information about:
 * - Actual clock frequencies
 * - PLL lock status
 * - Initialization status of each peripheral
 * - System uptime
 * 
 * Useful for:
 * - Verifying initialization succeeded
 * - Detecting clock problems
 * - System diagnostics
 * 
 * @return Pointer to diagnostics structure (static, valid for program lifetime)
 * 
 * @example
 * @code
 * const system_diagnostics_t *diag = system_get_diagnostics();
 * uart_printf("Core Clock: %u Hz\n", diag->core_clock_hz);
 * uart_printf("PLL Locked: %s\n", diag->pll_locked ? "yes" : "NO");
 * @endcode
 */
const system_diagnostics_t* system_get_diagnostics(void);

/**
 * @brief Print system diagnostics to UART (for debugging)
 * 
 * Outputs formatted diagnostics including:
 * - Clock frequencies (core, APB1, APB2)
 * - Peripheral initialization status
 * - Hardware configuration details
 * 
 * Useful during development and troubleshooting.
 * 
 * @note Requires UART1 to be initialized
 * @note Output disabled if SYSTEM_DIAGNOSTICS_ENABLED = 0
 * 
 * @example
 * @code
 * system_print_diagnostics();
 * // Output:
 * // [DIAG] Core Clock: 72000000 Hz (72 MHz)
 * // [DIAG] APB1 Clock: 36000000 Hz (36 MHz)
 * // [DIAG] PLL Locked: yes
 * // [DIAG] Peripherals: GPIO=ok, UART1=ok, UART2=ok, I2C=ok, PWM=ok
 * @endcode
 */
void system_print_diagnostics(void);

/**
 * @brief Get human-readable error message
 * 
 * Converts system_status_t error code to descriptive string.
 * 
 * @param status System status code
 * @return Pointer to static error message string
 * 
 * @note Returned string pointer valid for program lifetime
 * 
 * @example
 * @code
 * system_status_t err = system_init();
 * if (err != SYSTEM_OK) {
 *     uart_printf("Error: %s\n", system_error_string(err));
 * }
 * @endcode
 */
const char* system_error_string(system_status_t status);

/**
 * @brief LED control function
 * 
 * Simple on/off control for status LED (PC13).
 * 
 * @param on true to turn LED on, false to turn off
 */
void system_led_set(bool on);

/**
 * @brief Toggle LED (for blinking diagnostics)
 */
void system_led_toggle(void);

/** @} */

#endif  // SYSTEM_INIT_H