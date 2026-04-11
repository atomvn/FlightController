#ifndef DMA_H
#define DMA_H

#include <stdint.h>
#include <stdbool.h>
#include <libopencm3/stm32/f1/nvic.h>

/* ======================== Configuration ======================== */

/**
 * @brief S-BUS receiver frame size (fixed by protocol)
 * @note S-BUS protocol uses exactly 25-byte frames at ~14.5 ms intervals
 */
#define SBUS_FRAME_SIZE      25

/**
 * @brief Number of DMA buffers for ping-pong operation
 * @note Using 2 buffers allows DMA to fill one while ISR processes the other
 */
#define SBUS_DMA_BUFFERS     2

/**
 * @brief DMA controller and channel configuration
 * @note STM32F1: USART2_RX on DMA1_CHANNEL6
 */
#define SBUS_DMA_CONTROLLER  DMA1
#define SBUS_DMA_CHANNEL     DMA_CHANNEL6
#define SBUS_USART_NUM       2
#define SBUS_USART_DR        USART_DR(USART2)
#define SBUS_DMA_IRQ         NVIC_DMA1_CHANNEL6_IRQ

/* ======================== DMA Buffer Management ======================== */

/**
 * @brief S-BUS DMA statistics for diagnostics
 */
typedef struct {
    uint32_t total_frames;              ///< Total frames received
    uint32_t valid_frames;              ///< Frames that passed CRC check
    uint32_t overflow_count;            ///< Overflow errors detected
    uint32_t checksum_errors;           ///< CRC/checksum failures
    uint32_t last_frame_ticks;          ///< Tick count of last frame
    uint32_t min_frame_interval_ms;     ///< Minimum frame-to-frame interval
    uint32_t max_frame_interval_ms;     ///< Maximum frame-to-frame interval
} sbus_dma_stats_t;

/**
 * @brief S-BUS DMA state structure
 * 
 * Manages double-buffered DMA reception with frame synchronization.
 * Uses ping-pong buffering: DMA fills one buffer while ISR processes the other.
 */
typedef struct {
    // Buffer management:
    uint8_t buffer[SBUS_DMA_BUFFERS][SBUS_FRAME_SIZE];  ///< Double buffers
    volatile uint8_t write_index;                       ///< DMA writes to this buffer
    volatile uint8_t read_index;                        ///< ISR reads from this buffer
    volatile bool frame_ready;                          ///< true when frame in read buffer is complete
    
    // Statistics:
    sbus_dma_stats_t stats;
    
    // Configuration:
    bool initialized;                   ///< Guard against double initialization
} sbus_dma_state_t;

extern volatile sbus_dma_state_t sbus_dma;

/* ======================== Public API ======================== */

/**
 * @brief Initialize S-BUS DMA reception
 * 
 * Configures DMA1_CHANNEL6 for USART2 RX with frame-based reception.
 * Uses ping-pong buffering for safe data handoff between DMA and ISR.
 * 
 * @return 0 on success, -1 if already initialized or on error
 * 
 * @pre USART2 is initialized and running
 * @pre No other code is accessing sbus_dma buffer
 * @post DMA is running and receiving S-BUS frames
 * @post sbus_dma.frame_ready indicates when new frame available
 * 
 * @example
 * if (sbus_dma_init() != 0) {
 *     uart_printf("[ERROR] S-BUS DMA init failed\n");
 *     return;
 * }
 * uart_printf("[OK] S-BUS DMA ready\n");
 */
int32_t sbus_dma_init(void);

/**
 * @brief Disable S-BUS DMA reception
 * 
 * Stops DMA transfers and frees resources. Can be called to reinitialize.
 * 
 * @return 0 on success, -1 if not initialized
 * 
 * @post DMA is stopped
 * @post sbus_dma.initialized = false
 */
int32_t sbus_dma_disable(void);

/**
 * @brief Reinitialize S-BUS DMA (recovery from stuck condition)
 * 
 * Safely stops and restarts DMA. Used when frame sync is lost.
 * 
 * @return 0 on success, -1 on error
 * 
 * @post DMA is restarted from clean state
 */
int32_t sbus_dma_reinit(void);

/**
 * @brief Check if a new S-BUS frame is available
 * 
 * Non-blocking check to see if frame has been received and is ready for reading.
 * After reading, call sbus_dma_next_frame() to prepare for next frame.
 * 
 * @return true if frame is ready, false otherwise
 * 
 * @example
 * if (sbus_dma_frame_ready()) {
 *     uint8_t *frame = sbus_dma_get_frame();
 *     process_sbus_frame(frame);
 *     sbus_dma_next_frame();
 * }
 */
bool sbus_dma_frame_ready(void);

/**
 * @brief Get pointer to current S-BUS frame buffer
 * 
 * Returns pointer to the buffer containing the most recent complete frame.
 * Buffer remains valid until sbus_dma_next_frame() is called.
 * 
 * @return Pointer to SBUS_FRAME_SIZE byte frame, or NULL if no frame ready
 * 
 * @pre sbus_dma_frame_ready() returned true
 * 
 * @example
 * const uint8_t *frame = sbus_dma_get_frame();
 * if (frame) {
 *     uint16_t channel0 = ((frame[1] << 8) | frame[2]) & 0x7FF;
 * }
 */
const volatile uint8_t* sbus_dma_get_frame(void);

/**
 * @brief Advance to next S-BUS frame (swap read buffer)
 * 
 * After processing current frame, call this to prepare for next frame.
 * Marks current buffer as free for DMA to write into.
 * 
 * @return 0 on success, -1 if no frame was ready
 * 
 * @post Read buffer is swapped, frame_ready is reset
 * 
 * @example
 * uint8_t *frame = sbus_dma_get_frame();
 * process_frame(frame);
 * sbus_dma_next_frame();  // Ready for next
 */
int32_t sbus_dma_next_frame(void);

/**
 * @brief Get S-BUS DMA statistics
 * 
 * Returns diagnostic information about frame reception.
 * Useful for detecting reception issues (overflow, frame loss).
 * 
 * @return Pointer to statistics structure
 * 
 * @example
 * const sbus_dma_stats_t *stats = sbus_dma_get_stats();
 * uart_printf("Frames: %u, Errors: %u, Overflow: %u\n",
 *            stats->total_frames, stats->checksum_errors, stats->overflow_count);
 */
const volatile sbus_dma_stats_t* sbus_dma_get_stats(void);

/**
 * @brief Reset DMA statistics
 * 
 * Clears all error and frame counters.
 */
void sbus_dma_stats_reset(void);

#endif  // DMA_H
