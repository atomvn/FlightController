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

#include "libopencm3/cm3/common.h"
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/usart.h>

#include "driver/uart.h"
#include "dma.h"

/** @brief S-BUS DMA state variable */
sbus_dma_state_t g_sbus_dma_state = {
    .frame_ready = false,
    .initialized = false
};

/** @brief GPS DMA state variable */
gps_dma_state_t g_gps_dma_state = {
    .frame_ready = false,
    .initialized = false
};

static bool is_uart2_initialized(void) {
    // Check if USART2 is enabled and configured for RX
    return (RCC_APB1ENR & RCC_APB1ENR_USART2EN);
}

static bool is_uart3_initialized(void) {
    // Check if USART3 is enabled and configured for RX
    return (RCC_APB1ENR & RCC_APB1ENR_USART3EN);
}

/**
 * @brief Initialize S-BUS DMA reception
 * 
 * Configures DMA1_CHANNEL6 for USART2 RX with frame-based reception.
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
dma_error_t sbus_dma_init(void) {
    if (g_sbus_dma_state.initialized) {
        return DMA_INITIALIZED; // Already initialized
    }

    if (!(is_uart2_initialized())) {
        return DMA_UART_NOT_ENABLED; // USART2 not ready
    }

    /* clock enable DMA1*/
    rcc_periph_clock_enable(RCC_DMA1);

    /* reset dma*/
    dma_channel_reset(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);

    /* set source/peripheral address*/
    dma_set_peripheral_address(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL, (uint32_t)&SBUS_USART_DR);
    
    /* set destination/memory address*/
    dma_set_memory_address(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL, (uint32_t)g_sbus_dma_state.sbus_dma_buf);

    /* set number of data per transfer after interrupt happen*/
    dma_set_number_of_data(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL, SBUS_DMA_BUF_SIZE);

    /* set read from peripheral */
    dma_set_read_from_peripheral(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);

    /* set memory increment mode */
    dma_enable_memory_increment_mode(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);

    /* set peripheral and memory data size to 8-bit */
    dma_set_peripheral_size(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL, DMA_CCR_PSIZE_8BIT);

    /* set memory data size to 8-bit */
    dma_set_memory_size(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL, DMA_CCR_MSIZE_8BIT);

    /* enable circular mode for continuous reception */
    dma_enable_circular_mode(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);

    /* enable the DMA channel */
    dma_enable_channel(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);

    /* enable USART2 RX DMA */
    usart_enable_rx_dma(USART2);

    g_sbus_dma_state.initialized = true;
    return DMA_OK;
}

/** @brief deinit sbus dma */
dma_error_t sbus_dma_deinit(void) {
    if (!g_sbus_dma_state.initialized) {
        return DMA_NOT_INITIALIZED; // Not initialized
    }

    /* Disable USART2 RX DMA */
    usart_disable_rx_dma(USART2);

    /* Disable the DMA channel */
    dma_disable_channel(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);

    /* Reset DMA channel configuration */
    dma_channel_reset(SBUS_DMA_CONTROLLER, SBUS_DMA_CHANNEL);

    g_sbus_dma_state.initialized = false;
    return DMA_OK;
}

/** @brief Initialize GPS DMA reception
 * 
 * Configures DMA1_CHANNEL3 for USART3 RX with frame-based reception.
 * 
 * @return 0 on success, negative error code on failure
 * 
 * @pre USART3 is initialized and running
 * @pre No other code is accessing gps_dma buffer
 * @post DMA is running and receiving GPS data
 * @post gps_dma.frame_ready indicates when new frame available
 */
dma_error_t gps_dma_init(void) {
    if (g_gps_dma_state.initialized) {
        uart_printf("[WARN] GPS DMA already initialized\n");
        return DMA_INITIALIZED; // Already initialized
    }

    if (!(is_uart3_initialized())) {
        uart_printf("[ERROR] USART3 not initialized\n");
        return DMA_UART_NOT_ENABLED; // USART3 not ready
    }
    /* clock enable DMA1*/
    rcc_periph_clock_enable(RCC_DMA1);

    /* reset dma*/
    dma_channel_reset(GPS_DMA_CONTROLLER, GPS_DMA_CHANNEL);

    /* set source/peripheral address*/
    dma_set_peripheral_address(GPS_DMA_CONTROLLER, GPS_DMA_CHANNEL, (uint32_t)&GPS_USART_DR);

    /* set destination/memory address*/
    dma_set_memory_address(GPS_DMA_CONTROLLER, GPS_DMA_CHANNEL, (uint32_t)g_gps_dma_state.gps_dma_buf);

    /* set number of data per transfer after interrupt happen*/
    dma_set_number_of_data(GPS_DMA_CONTROLLER, GPS_DMA_CHANNEL, GPS_DMA_BUF_SIZE);

    /* set read from peripheral */
    dma_set_read_from_peripheral(GPS_DMA_CONTROLLER, GPS_DMA_CHANNEL);

    /* set memory increment mode */
    dma_enable_memory_increment_mode(GPS_DMA_CONTROLLER, GPS_DMA_CHANNEL);

    /* set peripheral and memory data size to 8-bit */
    dma_set_peripheral_size(GPS_DMA_CONTROLLER, GPS_DMA_CHANNEL, DMA_CCR_PSIZE_8BIT);

    /* set memory data size to 8-bit */
    dma_set_memory_size(GPS_DMA_CONTROLLER, GPS_DMA_CHANNEL, DMA_CCR_MSIZE_8BIT);

    /* enable circular mode for continuous reception */
    dma_enable_circular_mode(GPS_DMA_CONTROLLER, GPS_DMA_CHANNEL);

    /* enable the DMA channel */
    dma_enable_channel(GPS_DMA_CONTROLLER, GPS_DMA_CHANNEL);

    /* enable USART3 RX DMA */
    usart_enable_rx_dma(USART3);
    g_gps_dma_state.initialized = true;
    return DMA_OK;
}