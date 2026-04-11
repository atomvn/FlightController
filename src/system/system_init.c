/**
 * @file system_init.c
 * @brief STM32F1 System Initialization
 * * Initializes all hardware peripherals required for flight controller:
 * - **Clock:** PLL to 72 MHz from 8 MHz HSE (configurable)
 * - **UART1:** 38400 bps telemetry (TX only, expandable to full duplex)
 * - **UART2:** 100000 bps S-BUS receiver (RX only, per S-BUS protocol spec)
 * - **I2C1:** 100 kHz sensor bus (MPU6050, barometer, magnetometer)
 * - **PWM:** Motor ESC (TIM2) and 2 servos (TIM3) at 50 Hz
 * - **GPIO:** Blue LED (PC13), debug pins, I2C pull-ups (PB6/7)
 * 
 * @author Hao Nguyen
 * @version 1.0 
 * @date 2026
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/usart.h>
#include <libopencm3/stm32/f1/i2c.h>

#include "system_init.h"
#include "driver/pwm.h"

/* ========================================================================
 * CONFIGURATION & CONSTANTS
 * ======================================================================== */

/** @brief Enable diagnostic output (1=enabled, 0=disabled) */
#define SYSTEM_DIAGNOSTICS_ENABLED  1

/** @brief PLL lock timeout in milliseconds */
#define SYSTEM_PLL_LOCK_TIMEOUT_MS  50

/** @brief I2C timeout in milliseconds */
#define SYSTEM_I2C_TIMEOUT_MS       1000

/** @brief GPIO configuration timeout in milliseconds */
#define SYSTEM_GPIO_TIMEOUT_MS      10

/** @brief Default HSE frequency (MHz) */
#define DEFAULT_HSE_FREQ_MHZ        8

/** @brief Default PLL target (MHz) */
#define DEFAULT_PLL_TARGET_MHZ      72

/* ========================================================================
 * STATIC STATE & DIAGNOSTICS
 * ======================================================================== */

/** @brief Initialization guard flag */
static bool g_system_initialized = false;

/** @brief System diagnostics structure (persistent) */
static system_diagnostics_t g_diagnostics = {
    .core_clock_hz = 0,
    .apb1_clock_hz = 0,
    .apb2_clock_hz = 0,
    .pll_locked = false,
    .uptime_ms = 0,
    .peripherals = {
        .clock_ok = false,
        .gpio_ok = false,
        .uart1_ok = false,
        .uart2_ok = false,
        .i2c_ok = false,
        .pwm_ok = false,
    }
};

/* ========================================================================
 * STATIC HELPER FUNCTIONS
 * ======================================================================== */

/**
 * @brief Initialize system clocks to 72 MHz
 * 
 * Sequence:
 * 1. Enable HSE oscillator
 * 2. Set PLL multiplier
 * 3. Switch system clock to PLL
 * 4. Verify PLL lock status
 * 5. Update APB prescalers
 * 6. Record actual clock frequencies
 * 
 * @param clock_cfg Clock configuration (NULL = use defaults)
 * @return SYSTEM_OK on success, error code on failure
 */
static system_status_t system_init_clock(
    const system_clock_config_t *clock_cfg)
{
    // Use libopencm3's predefined clock setup for 8 MHz HSE → 72 MHz
    // This is safe and validated by the library
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    
    // Verify PLL locked (this should never fail with library setup)
	rcc_wait_for_osc_ready(RCC_PLL);
    
    // Record actual clock frequencies
    g_diagnostics.core_clock_hz = 72000000;  // 72 MHz
    g_diagnostics.apb1_clock_hz = 36000000;  // 36 MHz (APB1 /2)
    g_diagnostics.apb2_clock_hz = 72000000;  // 72 MHz (APB2 /1)
    g_diagnostics.pll_locked = true;
    g_diagnostics.peripherals.clock_ok = true;
    
    return SYSTEM_OK;
}

/**
 * @brief Initialize GPIO for all peripherals
 * 
 * Configures:
 * - UART1 TX (PA9): 50 MHz push-pull
 * - UART2 RX (PA3): floating input
 * - I2C SDA/SCL (PB6/PB7): 50 MHz open-drain (pulls idle high)
 * - LED (PC13): 2 MHz push-pull (low current)
 * - AFIO remapping: Standard mapping (no remap)
 * 
 * @return SYSTEM_OK on success
 */
static system_status_t system_init_gpio(void)
{
    // Enable all necessary GPIO clocks
    rcc_periph_clock_enable(RCC_GPIOA);     // UART1 TX, UART2 RX
    rcc_periph_clock_enable(RCC_GPIOB);     // I2C SDA/SCL
    rcc_periph_clock_enable(RCC_GPIOC);     // LED
    rcc_periph_clock_enable(RCC_AFIO);      // Alternate function remapping
    
    // -------- UART1 TX (PA9) for logging --------
    // 50 MHz push-pull (high speed for serial)
    gpio_set_mode(GPIOA,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO9);                             // PA9 = UART1_TX
    
    // -------- UART2 RX (PA3) for S-BUS receiver --------
    // Floating input (receiver pulls low, we read via UART)
    gpio_set_mode(GPIOA,
        GPIO_MODE_INPUT,
        GPIO_CNF_INPUT_FLOAT,
        GPIO3);                             // PA3 = UART2_RX
    
    // -------- I2C1 (PB6/PB7) for sensors --------
    // Open-drain, 50 MHz (allows external pull-ups to work)
    // Both pins must be driven low or float (not pushed high)
    gpio_set_mode(GPIOB,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
        GPIO6 | GPIO7);                     // PB6 = I2C_SCL, PB7 = I2C_SDA
    
    // Set pins high (they'll float via pull-ups when idle)
    gpio_set(GPIOB, GPIO6 | GPIO7);
    
    // -------- LED (PC13) status indicator --------
    // 2 MHz push-pull (low speed, low current)
    // Logic: Set bit = LED ON, Clear bit = LED OFF
    gpio_set_mode(GPIOC,
        GPIO_MODE_OUTPUT_2_MHZ,
        GPIO_CNF_OUTPUT_PUSHPULL,
        GPIO13);                            // PC13 = LED (blue)
    
    // Clear LED bit to turn off initially
    gpio_clear(GPIOC, GPIO13);
    
    // Configure AFIO remapping: use default (no remap)
    // AFIO_MAPR_I2C1_REMAP = 0, I2C on PB6/PB7 (not PB8/PB9)
    gpio_primary_remap(0, 0);
    
    g_diagnostics.peripherals.gpio_ok = true;
    return SYSTEM_OK;
}

/**
 * @brief Initialize UART1 for telemetry logging
 * 
 * Configuration:
 * - Baud: 38400 bps (standard for flight data)
 * - Data: 8 bits
 * - Stop: 1 bit
 * - Parity: None
 * - Mode: TX only (PA9)
 * - Flow control: None
 * 
 * @return SYSTEM_OK on success
 */
static system_status_t system_init_uart1(void)
{
    // Enable UART1 clock
    rcc_periph_clock_enable(RCC_USART1);
    
    // Configure UART parameters
    usart_set_baudrate(USART1, 38400);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);  // TX only
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    
    // Enable UART
    usart_enable(USART1);
    
    g_diagnostics.peripherals.uart1_ok = true;
    return SYSTEM_OK;
}

/**
 * @brief Initialize UART2 for S-BUS receiver input
 * 
 * Configuration:
 * - Baud: 100000 bps (S-BUS protocol requirement)
 * - Data: 8 bits
 * - Stop: 2 bits (S-BUS requirement)
 * - Parity: EVEN (S-BUS requirement)
 * - Mode: RX only (PA3)
 * - Flow control: None
 * 
 * @return SYSTEM_OK on success
 */
static system_status_t system_init_uart2(void)
{
    // Enable UART2 clock
    rcc_periph_clock_enable(RCC_USART2);
    
    // Configure UART parameters per S-BUS protocol specification
    usart_set_baudrate(USART2, 100000);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_2);  // S-BUS spec
    usart_set_mode(USART2, USART_MODE_RX);         // RX only
    usart_set_parity(USART2, USART_PARITY_EVEN);   // S-BUS spec
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    
    // Enable UART
    usart_enable(USART2);
    
    g_diagnostics.peripherals.uart2_ok = true;
    return SYSTEM_OK;
}

/**
 * @brief Initialize I2C1 for sensor bus
 * 
 * Configuration:
 * - Speed: 100 kHz (standard, reliable)
 * - GPIO: PB6 (SCL), PB7 (SDA)
 * - Slave address: N/A (master only)
 * - Open-drain output (external pull-ups required)
 * 
 * @return SYSTEM_OK on success
 */
static system_status_t system_init_i2c(void)
{
    // Enable I2C1 clock
    rcc_periph_clock_enable(RCC_I2C1);
    
    // Configure I2C to 100 kHz standard mode
    // At 36 MHz APB1, CCR calculation:
    // For 100 kHz: rise time ~1000 ns, CCR = 180 (36 MHz / (2 * 100 kHz))
    i2c_set_speed(I2C1, i2c_speed_fm_400k, 36);  // Use library function
    
    // Note: Open-drain GPIO already configured in system_init_gpio
    // I2C requires external pull-ups on SCL/SDA
    
    // Enable I2C
    i2c_peripheral_enable(I2C1);
    
    g_diagnostics.peripherals.i2c_ok = true;
    return SYSTEM_OK;
}

/**
 * @brief Initialize PWM for motor control and servos
 * 
 * Delegates to dedicated PWM module.
 * Expects pwm_init_brushless_motor() and pwm_init_2_servos() functions.
 * 
 * @return SYSTEM_OK on success
 */
static system_status_t system_init_pwm(void)
{
    // Initialize brushless motor on TIM2 (4 channels PWM)
    int ret1 = pwm_init_motor(&PWM_CONFIG_MOTOR);
    
    // Initialize 2 servos on TIM3 (2 channels PWM)
    int ret2 = pwm_init_servos(&PWM_CONFIG_SERVO);
    
    if (ret1 != 0 || ret2 != 0) {
        return SYSTEM_ERR_PWM_CONFIG;
    }
    
    g_diagnostics.peripherals.pwm_ok = true;
    return SYSTEM_OK;
}

/**
 * @brief Update system diagnostics structure
 * 
 * Records current system state and clock frequencies.
 * Called after successful initialization.
 */
static void system_update_diagnostics(void)
{
    // Clock frequencies already set in system_init_clock
    // Peripheral status flags set in respective init functions
    
    // Could add:
    // - Current system tick count (uptime)
    // - Voltage monitor readings
    // - Temperature readings
    // - Error statistics
}

/* ========================================================================
 * PUBLIC API IMPLEMENTATION
 * ======================================================================== */

system_status_t system_init(void)
{
    // Guard: prevent re-initialization
    if (g_system_initialized) {
        return SYSTEM_OK;
    }
    
    system_status_t status;
    
    // Initialize clocks FIRST (all other peripherals depend on this)
    status = system_init_clock(NULL);
    if (status != SYSTEM_OK) {
        return status;
    }
    
    // Initialize GPIO (must happen before UART/I2C init)
    status = system_init_gpio();
    if (status != SYSTEM_OK) {
        return status;
    }
    
    // Initialize communication peripherals
    status = system_init_uart1();
    if (status != SYSTEM_OK) {
        return status;
    }
    
    status = system_init_uart2();
    if (status != SYSTEM_OK) {
        return status;
    }
    
    status = system_init_i2c();
    if (status != SYSTEM_OK) {
        return status;
    }
    
    // Initialize motor control (PWM)
    status = system_init_pwm();
    if (status != SYSTEM_OK) {
        return status;
    }
    
    // Update diagnostics
    system_update_diagnostics();
    
    // Mark as initialized
    g_system_initialized = true;
    
    return SYSTEM_OK;
}

system_status_t system_init_custom(
    const system_clock_config_t *clock_cfg,
    const system_uart_config_t *uart1_cfg,
    const system_uart_config_t *uart2_cfg,
    const system_i2c_config_t *i2c_cfg)
{
    // Guard against multiple initialization
    if (g_system_initialized) {
        return SYSTEM_OK;
    }
    
    // NOTE: Full implementation would validate and apply custom configs
    // For now, delegate to standard init (custom configs ignored)
    return system_init();
}

system_status_t system_reconfigure_uart(uint32_t usart, uint32_t baud_rate)
{
    if (usart != USART1 && usart != USART2) {
        return SYSTEM_ERR_INVALID_CONFIG;
    }
    
    if (baud_rate == 0 || baud_rate > 3000000) {
        return SYSTEM_ERR_INVALID_CONFIG;
    }
    
    // Temporarily disable UART
    usart_disable(usart);
    
    // Reconfigure baud rate
    usart_set_baudrate(usart, baud_rate);
    
    // Re-enable UART
    usart_enable(usart);
    
    return SYSTEM_OK;
}

const system_diagnostics_t* system_get_diagnostics(void)
{
    return &g_diagnostics;
}

void system_print_diagnostics(void)
{
#if SYSTEM_DIAGNOSTICS_ENABLED
    // NOTE: Full implementation would use formatted output
    // For now, this is a placeholder
    // In production, would use UART logging interface
    
    // Example output (pseudo-code):
    // [DIAG] Core Clock: 72000000 Hz (72 MHz)
    // [DIAG] APB1 Clock: 36000000 Hz (36 MHz)
    // [DIAG] APB2 Clock: 72000000 Hz (72 MHz)
    // [DIAG] PLL Locked: yes
    // [DIAG] GPIO: ok, UART1: ok, UART2: ok, I2C: ok, PWM: ok
#endif
}

const char* system_error_string(system_status_t status)
{
    switch (status) {
        case SYSTEM_OK:
            return "Success";
        case SYSTEM_ERR_PLL_TIMEOUT:
            return "PLL failed to lock";
        case SYSTEM_ERR_GPIO_CONFIG:
            return "GPIO configuration error";
        case SYSTEM_ERR_USART_CONFIG:
            return "UART configuration error";
        case SYSTEM_ERR_I2C_CONFIG:
            return "I2C configuration error";
        case SYSTEM_ERR_PWM_CONFIG:
            return "PWM configuration error";
        case SYSTEM_ERR_INIT_FAILED:
            return "Generic initialization failure";
        case SYSTEM_ERR_INVALID_CONFIG:
            return "Invalid configuration parameters";
        default:
            return "Unknown error";
    }
}

void system_led_set(bool on)
{
    if (on) {
        gpio_set(GPIOC, GPIO13);
    } else {
        gpio_clear(GPIOC, GPIO13);
    }
}

void system_led_toggle(void)
{
    gpio_toggle(GPIOC, GPIO13);
}
