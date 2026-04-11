/**
 * @file mcre7_v2.h
 * @brief S-BUS Receiver Module - RC Channel Decoding
 *
 * @details
 * Processes Futaba S-BUS protocol (25-byte frames, 100 kHz baud).
 * Decodes 16 RC channels with signal loss and failsafe detection.
 * Provides thread-safe access to channel data via FreeRTOS mutex.
 * 
 * **S-BUS Protocol Specifications:**
 * - Baud Rate: 100,000 bps (non-standard, inverted signal)
 * - Frame Size: 25 bytes total
 * - RC Channels: 16 channels, 11 bits each
 * - Channel Range: 0-2047 (corresponds to 1000-2000 µs pulse width)
 * - Center Position: 992 (corresponds to 1500 µs pulse width)
 * - Frame Rate: ~14.5 ms per frame (~68 Hz update rate)
 * 
 * **Frame Structure (25 bytes):**
 * ```
 * Byte 0:     0x0F (header/start byte)
 * Bytes 1-22: 16 RC channels packed into 22 bytes (11 bits each)
 * Byte 23:    Flags (bit 3=signal loss, bit 2=failsafe, bit 0=frame lost)
 * Byte 24:    0x00 (end byte/footer)
 * ```
 *
 * **Channel Data Extraction (Byte positions and bit math):**
 * - Channel 0:  bits [10:0] of bytes [1:0] (byte 1 bits 0-7, byte 2 bits 0-2)
 * - Channel 1:  bits [10:0] of bytes [2:1] (byte 2 bits 3-7, byte 3 bits 0-4)
 * - ... (pattern continues for all 16 channels)
 * See SBUS_EXTRACT_CHx macros for bit extraction details.
 *
 * **Usage Example:**
 * @code
 * // Initialize S-BUS decoder (call once in main)
 * sbus_status_t err = sbus_init();
 * if (err != SBUS_OK) {
 *     uart_printf("[FATAL] S-BUS init failed: %s\n", sbus_error_string(err));
 *     for (;;) __asm__("wfi");
 * }
 * 
 * // In reading/processing task (called periodically)
 * uint16_t channels[SBUS_NUM_CHANNELS];
 * sbus_status_t status = sbus_read_channels(channels, pdMS_TO_TICKS(100));
 * 
 * if (status == SBUS_OK) {
 *     // Success - channels contain latest RC input
 *     uint16_t throttle = channels[2];      // Channel 3 (0-indexed)
 *     uint16_t roll = channels[0];          // Channel 1
 *     uint16_t pitch = channels[1];         // Channel 2
 *     uint16_t yaw = channels[3];           // Channel 4
 *     
 *     uart_printf("CH1=%u CH2=%u CH3=%u CH4=%u\n", 
 *         channels[0], channels[1], channels[2], channels[3]);
 * } else if (status == SBUS_ERR_TIMEOUT) {
 *     // Signal lost - engage failsafe
 *     uart_printf("[WARN] S-BUS signal lost - no frame for >200ms\n");
 *     motor_control_failsafe();
 * } else {
 *     // Other error
 *     uart_printf("[ERROR] S-BUS error: %s\n", sbus_error_string(status));
 * }
 * @endcode
 *
 * **Thread Safety:**
 * - All public functions are thread-safe using FreeRTOS mutex
 * - Safe to call from multiple tasks or from ISR context
 * - Data is copied atomically to prevent partial reads of channel updates
 *
 * **Error Handling:**
 * All functions return sbus_status_t error codes:
 * - SBUS_OK (0) = operation succeeded
 * - Negative values = specific error conditions (see enum)
 * Use sbus_error_string() to get human-readable error messages.
 *
 * **Diagnostics:**
 * Call sbus_get_stats() to access frame reception statistics:
 * - total_frames: Total frames received since initialization
 * - valid_frames: Frames that passed validation
 * - signal_loss_count: Number of signal loss events
 * - last_update_tick: Timestamp of most recent successful frame
 *
 * **Performance Characteristics:**
 * - Frame processing: <100 µs
 * - Mutex acquisition: <10 µs (no contention)
 * - Memory: ~100 bytes (fixed allocation)
 * - Stack usage: ~64 bytes per task
 *
 * @author Hao Nguyen
 * @version 1.0 
 * @date 2026
 * 
 * @defgroup SBUS S-BUS Receiver Module
 * @{
 */

#ifndef MCRE7_V2_H
#define MCRE7_V2_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ========================================================================
 * S-BUS PROTOCOL CONSTANTS
 * ======================================================================== */

/** @brief S-BUS frame is exactly 25 bytes */
#define SBUS_FRAME_SIZE         25

/** @brief S-BUS frame header byte (start marker) */
#define SBUS_FRAME_HEADER       0x0F

/** @brief S-BUS frame footer byte (end marker) */
#define SBUS_FRAME_FOOTER       0x00

/** @brief Number of RC channels in S-BUS frame */
#define SBUS_NUM_CHANNELS       16

/** @brief Bits per RC channel (11-bit values) */
#define SBUS_CHANNEL_BITS       11

/** @brief RC channel value range: minimum (1000 µs pulse) */
#define SBUS_CHANNEL_MIN        192

/** @brief RC channel value range: maximum (2000 µs pulse) */
#define SBUS_CHANNEL_MAX        1792

/** @brief RC channel value range: center (1500 µs pulse, neutral) */
#define SBUS_CHANNEL_CENTER     992

/** @brief S-BUS signal loss flag (bit 3 of flags byte) */
#define SBUS_FLAG_SIGNAL_LOSS   (1 << 3)

/** @brief S-BUS failsafe flag (bit 2 of flags byte) */
#define SBUS_FLAG_FAILSAFE      (1 << 2)

/** @brief S-BUS frame lost flag (bit 0 of flags byte) */
#define SBUS_FLAG_FRAME_LOST    (1 << 0)

/** @brief Maximum time (ms) without frame before timeout (signal loss) */
#define SBUS_TIMEOUT_MS         200

/** @brief Update rate (Hz) - S-BUS nominally sends ~68 frames/sec */
#define SBUS_UPDATE_RATE_HZ     68

/** @brief S-BUS nominal frame interval (ms) - 1000/68 ≈ 14.7 ms */
#define SBUS_FRAME_INTERVAL_MS  15

/* ========================================================================
 * STATUS / ERROR CODES
 * ======================================================================== */

/**
 * @brief S-BUS status and error codes
 * 
 * Non-negative values indicate success, negative values indicate errors.
 * Can be converted to human-readable strings via sbus_error_string().
 */
typedef int32_t sbus_status_t;

#define SBUS_OK                    0      ///< Operation successful
#define SBUS_ERR_INVALID_FRAME     (-1)   ///< Frame header/footer invalid
#define SBUS_ERR_CRC_FAILED        (-2)   ///< CRC/checksum verification failed
#define SBUS_ERR_NO_FRAME          (-3)   ///< No frame available yet
#define SBUS_ERR_TIMEOUT           (-4)   ///< No frames for >SBUS_TIMEOUT_MS
#define SBUS_ERR_INIT_FAILED       (-5)   ///< Initialization failed
#define SBUS_ERR_INVALID_CONFIG    (-6)   ///< Invalid configuration parameters
#define SBUS_ERR_MUTEX_FAILED      (-7)   ///< FreeRTOS mutex error

/* ========================================================================
 * DATA STRUCTURES
 * ======================================================================== */

/**
 * @brief S-BUS frame with decoded metadata
 * 
 * Contains decoded RC channel values and status flags from S-BUS frame.
 */
typedef struct {
    uint16_t channels[SBUS_NUM_CHANNELS];  ///< 16 RC channels (11-bit values 0-2047)
    uint8_t flags;                         ///< Raw flags byte from S-BUS frame
    bool signal_loss;                      ///< true if receiver signal lost
    bool failsafe;                         ///< true if receiver in failsafe mode
    bool frame_lost;                       ///< true if frame lost detected
} sbus_frame_t;

/**
 * @brief S-BUS statistics for diagnostics and debugging
 * 
 * Useful for monitoring S-BUS link quality and detecting issues.
 */
typedef struct {
    uint32_t total_frames;                  ///< Total frames received
    uint32_t valid_frames;                  ///< Frames that passed all validation
    uint32_t invalid_frames;                ///< Frames with bad header/footer
    uint32_t crc_errors;                    ///< Frames with CRC failures
    uint32_t signal_loss_count;             ///< Number of signal loss events detected
    uint32_t failsafe_count;                ///< Number of failsafe activations
    uint32_t last_update_tick;              ///< FreeRTOS tick count of last valid frame
    uint32_t min_frame_interval_ms;         ///< Minimum observed frame interval
    uint32_t max_frame_interval_ms;         ///< Maximum observed frame interval
} sbus_stats_t;

/* ========================================================================
 * BIT EXTRACTION HELPER MACROS
 * ======================================================================== */

/**
 * @brief Extract RC channel 0 (11 bits) from S-BUS frame
 * 
 * S-BUS packs 16 channels (11-bit values) into 22 bytes using bit manipulation.
 * Channels are extracted using shifts and masks as defined by the protocol.
 * 
 * @param buf Pointer to S-BUS frame (must be SBUS_FRAME_SIZE bytes)
 * @return 11-bit channel value (0-2047)
 */
#define SBUS_EXTRACT_CH0(buf)   ((buf[1] | (buf[2] << 8)) & 0x07FF)
#define SBUS_EXTRACT_CH1(buf)   (((buf[2] >> 3) | (buf[3] << 5)) & 0x07FF)
#define SBUS_EXTRACT_CH2(buf)   (((buf[3] >> 6) | (buf[4] << 2) | (buf[5] << 10)) & 0x07FF)
#define SBUS_EXTRACT_CH3(buf)   (((buf[5] >> 1) | (buf[6] << 7)) & 0x07FF)
#define SBUS_EXTRACT_CH4(buf)   (((buf[6] >> 4) | (buf[7] << 4)) & 0x07FF)
#define SBUS_EXTRACT_CH5(buf)   (((buf[7] >> 7) | (buf[8] << 1) | (buf[9] << 9)) & 0x07FF)
#define SBUS_EXTRACT_CH6(buf)   (((buf[9] >> 2) | (buf[10] << 6)) & 0x07FF)
#define SBUS_EXTRACT_CH7(buf)   (((buf[10] >> 5) | (buf[11] << 3)) & 0x07FF)
#define SBUS_EXTRACT_CH8(buf)   ((buf[12] | (buf[13] << 8)) & 0x07FF)
#define SBUS_EXTRACT_CH9(buf)   (((buf[13] >> 3) | (buf[14] << 5)) & 0x07FF)
#define SBUS_EXTRACT_CH10(buf)  (((buf[14] >> 6) | (buf[15] << 2) | (buf[16] << 10)) & 0x07FF)
#define SBUS_EXTRACT_CH11(buf)  (((buf[16] >> 1) | (buf[17] << 7)) & 0x07FF)
#define SBUS_EXTRACT_CH12(buf)  (((buf[17] >> 4) | (buf[18] << 4)) & 0x07FF)
#define SBUS_EXTRACT_CH13(buf)  (((buf[18] >> 7) | (buf[19] << 1) | (buf[20] << 9)) & 0x07FF)
#define SBUS_EXTRACT_CH14(buf)  (((buf[20] >> 2) | (buf[21] << 6)) & 0x07FF)
#define SBUS_EXTRACT_CH15(buf)  (((buf[21] >> 5) | (buf[22] << 3)) & 0x07FF)

/* ========================================================================
 * PUBLIC API
 * ======================================================================== */

/**
 * @brief Initialize S-BUS receiver module
 * 
 * Initializes DMA reception, mutex for thread safety, and resets statistics.
 * Must be called once before using any other S-BUS functions.
 * 
 * **Performs:**
 * - Initializes S-BUS DMA reception (UART2, 100k baud already configured by system_init)
 * - Creates FreeRTOS mutex for thread-safe data access
 * - Resets frame statistics
 * - Sets up frame validation
 * 
 * @return SBUS_OK on success, error code on failure
 * 
 * @pre system_init() has been called (UART2 initialized)
 * @pre FreeRTOS scheduler not yet started (or called from main before vTaskStartScheduler)
 * @post S-BUS receiver is ready for frame reception
 * @post All frames received will be validated and stored
 * 
 * @example
 * @code
 * int main(void) {
 *     system_init();  // Initialize clocks, GPIO, UART
 *     
 *     sbus_status_t err = sbus_init();
 *     if (err != SBUS_OK) {
 *         uart_printf("[FATAL] S-BUS init failed: %s\n", sbus_error_string(err));
 *         for (;;) __asm__("wfi");  // Halt
 *     }
 *     
 *     uart_printf("[OK] S-BUS ready\n");
 *     vTaskStartScheduler();
 *     return 0;
 * }
 * @endcode
 */
sbus_status_t sbus_init(void);

/**
 * @brief Read RC channel values (thread-safe)
 * 
 * Atomically reads the 16 RC channel values into provided buffer.
 * Uses FreeRTOS mutex to ensure safe access from multiple tasks.
 * 
 * **Checks:**
 * - Data freshness (returns error if no update for >SBUS_TIMEOUT_MS)
 * - Mutex acquisition within specified timeout
 * - Signal loss/failsafe flags
 * 
 * @param channels_out Pointer to 16-element buffer (must be non-NULL)
 * @param timeout_ms FreeRTOS timeout in ms (pdMS_TO_TICKS converts to ticks)
 * 
 * @return SBUS_OK if channels updated successfully
 * @return SBUS_ERR_TIMEOUT if no frames received for >SBUS_TIMEOUT_MS (signal loss)
 * @return SBUS_ERR_INVALID_CONFIG if channels_out is NULL
 * @return SBUS_ERR_MUTEX_FAILED if mutex acquisition times out
 * 
 * @pre sbus_init() has been called
 * @pre channels_out points to buffer with space for SBUS_NUM_CHANNELS uint16_t values
 * @post channels_out contains latest 16 RC channel values (11-bit, 0-2047 range)
 * 
 * @example
 * @code
 * uint16_t rc_channels[SBUS_NUM_CHANNELS];
 * sbus_status_t status = sbus_read_channels(rc_channels, pdMS_TO_TICKS(100));
 * 
 * if (status == SBUS_OK) {
 *     // Successfully read channels
 *     uint16_t throttle = rc_channels[2];  // Channel 3 (0-indexed)
 *     uint16_t roll = rc_channels[0];      // Channel 1
 * } else if (status == SBUS_ERR_TIMEOUT) {
 *     // Signal lost - failsafe mode
 *     uart_printf("[WARN] S-BUS signal lost\n");
 * }
 * @endcode
 */
sbus_status_t sbus_read_channels(
    uint16_t* channels_out,
    uint32_t timeout_ms);

/**
 * @brief Read last decoded S-BUS frame with metadata
 * 
 * Returns complete frame data including signal loss and failsafe flags.
 * More detailed than sbus_read_channels() - includes status information.
 * 
 * @param frame_out Pointer to sbus_frame_t structure (must be non-NULL)
 * @param timeout_ms FreeRTOS timeout in ms
 * 
 * @return SBUS_OK on success
 * @return SBUS_ERR_TIMEOUT if signal lost (no frame for >SBUS_TIMEOUT_MS)
 * @return SBUS_ERR_INVALID_CONFIG if frame_out is NULL
 * 
 * @post frame_out->channels contains 16 RC channel values
 * @post frame_out->signal_loss indicates receiver signal status
 * @post frame_out->failsafe indicates receiver failsafe mode
 * 
 * @example
 * @code
 * sbus_frame_t frame;
 * if (sbus_read_frame(&frame, pdMS_TO_TICKS(50)) == SBUS_OK) {
 *     if (frame.signal_loss) {
 *         uart_printf("[WARN] Receiver signal lost\n");
 *     }
 *     if (frame.failsafe) {
 *         uart_printf("[ALERT] Receiver in failsafe mode\n");
 *     }
 * }
 * @endcode
 */
sbus_status_t sbus_read_frame(
    sbus_frame_t* frame_out,
    uint32_t timeout_ms);

/**
 * @brief Get S-BUS reception statistics for diagnostics
 * 
 * Returns accumulated statistics since initialization (never reset during operation).
 * Useful for monitoring link quality and detecting recurring issues.
 * 
 * @return Pointer to static sbus_stats_t structure
 * 
 * @note Returned pointer is valid for program lifetime
 * @note Structure is updated in real-time by DMA ISR
 * 
 * @example
 * @code
 * const sbus_stats_t *stats = sbus_get_stats();
 * uint32_t valid_percent = (stats->valid_frames * 100) / stats->total_frames;
 * uart_printf("S-BUS Quality: %u%% (%u/%u frames valid)\n",
 *     valid_percent, stats->valid_frames, stats->total_frames);
 * @endcode
 */
const sbus_stats_t* sbus_get_stats(void);

/**
 * @brief Get human-readable error message
 * 
 * Converts sbus_status_t error code to descriptive string.
 * Useful for error reporting and debugging.
 * 
 * @param status Error code returned from S-BUS function
 * @return Pointer to static error string (valid for program lifetime)
 * 
 * @example
 * @code
 * sbus_status_t err = sbus_read_channels(channels, 10);
 * if (err != SBUS_OK) {
 *     uart_printf("[ERROR] %s\n", sbus_error_string(err));
 * }
 * @endcode
 */
const char* sbus_error_string(sbus_status_t status);

/**
 * @brief Convert S-BUS channel value to microseconds (pulse width)
 * 
 * Converts 11-bit S-BUS channel value (0-2047) to servo pulse width in microseconds.
 * Formula: pulse_us = (channel_value - 992) * 0.62 + 1500 ≈ channel_value * 0.62 + 988
 * 
 * @param channel_value S-BUS channel value (0-2047 typical, 192-1792 safe range)
 * @return Pulse width in microseconds (~1000-2000 µs for typical range)
 * 
 * @example
 * @code
 * uint16_t sbus_val = 992;      // Center position
 * uint16_t pulse_us = sbus_to_microseconds(sbus_val);  // Returns ~1500 µs
 * @endcode
 */
uint16_t sbus_to_microseconds(uint16_t channel_value);

/**
 * @brief FreeRTOS task for S-BUS reception and processing
 * 
 * Monitors DMA frame reception and validates incoming frames.
 * Can be created as independent task or called from application task.
 * 
 * **Should be created with:**
 * - Stack: configMINIMAL_STACK_SIZE + 64 bytes
 * - Priority: tskIDLE_PRIORITY + 1 (low priority, triggered by DMA)
 * - Parameters: NULL
 * 
 * @param params FreeRTOS task parameter (unused, cast to void)
 * 
 * @note This function never returns (infinite loop)
 * @note Debug output controlled by SBUS_DEBUG_ENABLED macro
 * 
 * @example
 * @code
 * xTaskCreate(
 *     mcre7_v2_task,
 *     "S-BUS Reader",
 *     configMINIMAL_STACK_SIZE + 64,
 *     NULL,
 *     tskIDLE_PRIORITY + 1,
 *     NULL);
 * @endcode
 */
void mcre7_v2_task(void* params);

/** @} */  // End SBUS group

#endif  // MCRE7_V2_H