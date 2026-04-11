/**
 * @file dma.c
 * @brief STM32F1 DMA Driver Implementation 
 *
 * @details
 *
 * @author Hao Nguyen
 * @version 1.0
 * @date 2026
 */
#include <FreeRTOS.h>
#include <task.h>
#include <string.h>

#include "libopencm3/cm3/common.h"
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/usart.h>

#include "driver/uart.h"
#include "dma.h"

/* ======================== Global State ======================== */

/**
 * @brief Global S-BUS DMA state structure
 * 
 * Managed by this module - external code accesses via getter functions.
 * Protected by taskENTER_CRITICAL() when modifying from ISR context.
 */
volatile sbus_dma_state_t sbus_dma = {
    .write_index = 0,
    .read_index = 1,  // Start with different indices
    .frame_ready = false,
    .initialized = false,
    .stats = {0}
};

/* ======================== DMA Interrupt Handler ======================== */

/**
 * @brief DMA1 Channel 6 interrupt handler (S-BUS frame complete)
 * 
 * Called by hardware when one SBUS_FRAME_SIZE bytes have been transferred.
 * Swaps write buffer and detects overflow conditions.
 */
void dma1_channel6_isr(void) {

    // Disable DMA channel to safely access buffer and update state
    dma_disable_channel(DMA1, DMA_CHANNEL6);

    // Check for hardware error flags:
    // TEIF6 = Transfer Error
    if (DMA_ISR(DMA1) & DMA_ISR_TEIF6) {
        uart_printf("[ERROR] DMA1_CH6 error (ISR=0x%x)\n", DMA_ISR(DMA1));
        sbus_dma.stats.overflow_count++;
        
        // Clear error flags:
        DMA_IFCR(DMA1) = DMA_IFCR_CTEIF6 | DMA_IFCR_CHTIF6;
        DMA_IFCR(DMA1) = DMA_IFCR_CTCIF6;  // Clear transfer complete
        return;
    }
    
    // Clear transfer complete interrupt flag:
    DMA_IFCR(DMA1) = DMA_IFCR_CTCIF6;
    
    // Statistics: Check frame arrival interval (detect overflow):
    uint32_t now = xTaskGetTickCount();
    uint32_t elapsed_ms = (now - sbus_dma.stats.last_frame_ticks) * portTICK_PERIOD_MS;
    
    sbus_dma.stats.last_frame_ticks = now;
    
    // S-BUS nominal rate is 14.5 ms per frame
    // Acceptable range: 10-20 ms
    if (sbus_dma.stats.total_frames > 0) {  // Skip first frame
        if (elapsed_ms < 10 || elapsed_ms > 20) {
            uart_printf("[WARN] S-BUS frame interval %u ms (expected ~14.5)\n", elapsed_ms);
            sbus_dma.stats.overflow_count++;
        }
        
        if (elapsed_ms < sbus_dma.stats.min_frame_interval_ms || 
            sbus_dma.stats.min_frame_interval_ms == 0) {
            sbus_dma.stats.min_frame_interval_ms = elapsed_ms;
        }
        
        if (elapsed_ms > sbus_dma.stats.max_frame_interval_ms) {
            sbus_dma.stats.max_frame_interval_ms = elapsed_ms;
        }
    }

    // Mark frame as ready:
    sbus_dma.frame_ready = true;
    sbus_dma.stats.total_frames++;

    // Swap read/write buffers:
    sbus_dma.read_index = sbus_dma.write_index;

    // Ping-pong: swap buffers for next DMA transfer
    sbus_dma.write_index = (sbus_dma.write_index + 1) % SBUS_DMA_BUFFERS;
    
    // Reconfigure DMA to fill the next buffer
    dma_set_memory_address(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL,
                          (uint32_t)&sbus_dma.buffer[sbus_dma.write_index][0]);
    
    // Reset the number of data to transfer for the next frame
    dma_set_number_of_data(DMA1, DMA_CHANNEL6, SBUS_FRAME_SIZE);

    // Re-enable DMA channel for next transfer
    dma_enable_channel(DMA1, DMA_CHANNEL6);
}

/* ======================== Initialization ======================== */

int32_t sbus_dma_init(void) {
    if (sbus_dma.initialized) {
        uart_printf("[WARN] S-BUS DMA already initialized\n");
        return -1;
    }
    
    // Validate USART2 is available:
    if (!(RCC_APB1ENR & RCC_APB1ENR_USART2EN)) {
        uart_printf("[ERROR] USART2 not enabled - cannot init DMA\n");
        return -1;
    }
    
    uart_printf("[LOG] Initializing S-BUS DMA: %d buffers × %d bytes\n",
               SBUS_DMA_BUFFERS, SBUS_FRAME_SIZE);
    
    // Enable DMA clock:
    rcc_periph_clock_enable(RCC_DMA1);
    
    // Reset channel (clear any stuck state):
    dma_channel_reset(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);
    
    // Configure DMA transfer:
    // - Source: USART2 DR register (peripheral)
    // - Dest: Our buffer (memory)
    // - Data size: SBUS_FRAME_SIZE bytes per transfer
    
    dma_set_peripheral_address(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL,
                              (uint32_t)&SBUS_USART_DR);
    
    dma_set_memory_address(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL,
                          (uint32_t)&sbus_dma.buffer[0][0]);
    
    // Set number of data to transfer (must be set before enabling channel)
    dma_set_number_of_data(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL, SBUS_FRAME_SIZE);
    
    // Configuration:
    dma_set_read_from_peripheral(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);
    dma_enable_memory_increment_mode(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);  // Increment memory
    dma_disable_peripheral_increment_mode(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);  // Fixed DR
    
    dma_set_peripheral_size(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL, DMA_CCR_MSIZE_8BIT);
    
    // Use circular mode: DMA automatically reloads and wraps at frame size
    // dma_enable_circular_mode(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);
    
    // Enable transfer complete interrupt:
    dma_enable_transfer_complete_interrupt(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);
    
    // Enable DMA channel:
    dma_enable_channel(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);
    
    // Enable USART2 to use DMA for RX:
    usart_enable_rx_dma(USART2);
    
    // Enable DMA interrupt in NVIC:
    nvic_enable_irq(SBUS_DMA_IRQ);
    
    // Initialize state:
    sbus_dma.initialized = true;
    sbus_dma.write_index = 0;
    sbus_dma.read_index = 1;
    sbus_dma.frame_ready = false;
    memset((void*)&sbus_dma.stats, 0, sizeof(sbus_dma.stats));
    
    uart_printf("[OK] S-BUS DMA ready: %d-byte frames, ~14.5ms interval\n", SBUS_FRAME_SIZE);
    
    return 0;
}

int32_t sbus_dma_disable(void) {
    if (!sbus_dma.initialized) {
        uart_printf("[WARN] S-BUS DMA not initialized\n");
        return -1;
    }
    
    // Disable hardware:
    dma_disable_channel(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);
    usart_disable_rx_dma(USART2);
    // nvic_disable_irq(SBUS_DMA_IRQ);
    
    // Clear state:
    sbus_dma.initialized = false;
    sbus_dma.frame_ready = false;
    
    uart_printf("[OK] S-BUS DMA disabled\n");
    
    return 0;
}

int32_t sbus_dma_reinit(void) {
    sbus_dma_disable();
    vTaskDelay(pdMS_TO_TICKS(10));  // Brief delay to ensure clean shutdown
    return sbus_dma_init();
}

/* ======================== Frame Access ======================== */

bool sbus_dma_frame_ready(void) {
    return sbus_dma.frame_ready;
}

const volatile uint8_t* sbus_dma_get_frame(void) {
    if (!sbus_dma.frame_ready) {
        return NULL;
    }
    
    return &sbus_dma.buffer[sbus_dma.read_index][0];
}

int32_t sbus_dma_next_frame(void) {
    if (!sbus_dma.frame_ready) {
        return -1;
    }
    
    // Critical section: Prevent ISR from accessing read_index during swap
    taskENTER_CRITICAL();
    // Reset ready flag:
    sbus_dma.frame_ready = false;
    taskEXIT_CRITICAL();
    
    return 0;
}

/* ======================== Diagnostics ======================== */

const volatile sbus_dma_stats_t* sbus_dma_get_stats(void) {
    return &sbus_dma.stats;
}

void sbus_dma_stats_reset(void) {
    taskENTER_CRITICAL();
    memset((void*)&sbus_dma.stats, 0, sizeof(sbus_dma.stats));
    taskEXIT_CRITICAL();
    
    uart_printf("[OK] S-BUS DMA statistics reset\n");
}
