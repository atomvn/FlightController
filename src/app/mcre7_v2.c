/**
 * @file mcre7_v2.c
 * @brief MCRE7 V2 S-BUS Receiver Task Implementation
 *
 * @details
 * Implements the MCRE7 V2 task which reads S-BUS frames from UART2 using DMA.
 * Provide API to read RC channel values with thread safety.
 * 
 * @author Hao Nguyen
 * @version 1.0
 * @date 2026
 */
#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/gpio.h>

#include "mcre7_v2.h"
#include "driver/uart.h"

#define IS_MCRE7_DEBUG_ENABLED 1

sbus_mcre7_t g_sbus_mcre7 = {
    .mutex = NULL
};

/** @brief Decode S-BUS frame */
static void sbus_decode(volatile uint8_t* buf) {
    g_sbus_mcre7.rc_channels[0]  = (buf[1]        | buf[2]  << 8) & 0x07FF;
    g_sbus_mcre7.rc_channels[1]  = (buf[2]  >> 3  | buf[3]  << 5) & 0x07FF;
    g_sbus_mcre7.rc_channels[2]  = (buf[3]  >> 6  | buf[4]  << 2  | buf[5] << 10) & 0x07FF;
    g_sbus_mcre7.rc_channels[3]  = (buf[5]  >> 1  | buf[6]  << 7) & 0x07FF;
    g_sbus_mcre7.rc_channels[4]  = (buf[6]  >> 4  | buf[7]  << 4) & 0x07FF;
    g_sbus_mcre7.rc_channels[5]  = (buf[7]  >> 7  | buf[8]  << 1  | buf[9] << 9) & 0x07FF;
    g_sbus_mcre7.rc_channels[6]  = (buf[9]  >> 2  | buf[10] << 6) & 0x07FF;
    g_sbus_mcre7.rc_channels[7]  = (buf[10] >> 5  | buf[11] << 3) & 0x07FF;
}

/** @brief Parse S-BUS frames from DMA buffer*/
static void sbus_parse(void) {
    for(int i=0; i<SBUS_DMA_BUF_SIZE-25; i++) {
        if (g_sbus_dma_state.sbus_dma_buf[i] == 0x0F && g_sbus_dma_state.sbus_dma_buf[i+24] == 0x00) {
            sbus_decode(&g_sbus_dma_state.sbus_dma_buf[i]);
        }
    }
}

/** @brief Initialize MCRE7 V2 task and resources */
mcre7_v2_error_t mcre7_v2_init(void) {
    if (g_sbus_dma_state.initialized) {
        return MCRE7_V2_OK; // Already initialized, not an error
    }

    dma_error_t dma_err = sbus_dma_init();
    if (dma_err != DMA_OK) {
        return MCRE7_V2_DMA_ERROR; // DMA initialization failed
    }
    
    g_sbus_mcre7.mutex = xSemaphoreCreateMutex();
    if (g_sbus_mcre7.mutex == NULL) {
        return MCRE7_V2_MUTEX_ERROR; // Failed to create mutex
    }

    #if IS_DEBUG_ENABLED
    uart_printf("[OK] MCRE7 V2 initialized successfully\n");
    #endif
    return MCRE7_V2_OK;
}

/** @brief Read MCRE7 V2 RC channels with thread safety */
mcre7_v2_error_t read_mcre7_v2_channels(uint16_t* channels, uint32_t timeout) {
    if (channels == NULL) {
        return MCRE7_V2_INVALID_ARG; // Invalid argument
    }

    if (xSemaphoreTake(g_sbus_mcre7.mutex, pdMS_TO_TICKS(timeout)) == pdTRUE) {
        for (int i = 0; i < 16; i++) {
            channels[i] = g_sbus_mcre7.rc_channels[i];
        }
        xSemaphoreGive(g_sbus_mcre7.mutex);
        return MCRE7_V2_OK;
    } else {
        return MCRE7_V2_MUTEX_ERROR; // Failed to acquire mutex within timeout
    }
}

/** @brief MCRE7 V2 task function */
void mcre7_v2_task(void* params){
    (void)params;
    mcre7_v2_error_t err = mcre7_v2_init();
    if (err != MCRE7_V2_OK) {
        uart_printf("[ERROR] MCRE7 V2 initialization failed with code: %d\n", err);
        while(1);
    }

    while(1) {
        sbus_parse();
        #if IS_MCRE7_DEBUG_ENABLED
        uart_send_string("***********************MCRE data*********************\n");
        uart_printf("Channel 1: %u\n", (uint32_t)g_sbus_mcre7.rc_channels[0]);
        uart_printf("Channel 2: %u\n", (uint32_t)g_sbus_mcre7.rc_channels[1]);
        uart_printf("Channel 3: %u\n", (uint32_t)g_sbus_mcre7.rc_channels[2]);
        uart_printf("Channel 4: %u\n", (uint32_t)g_sbus_mcre7.rc_channels[3]);
        uart_printf("Channel 5: %u\n", (uint32_t)g_sbus_mcre7.rc_channels[4]);
        uart_printf("Channel 6: %u\n", (uint32_t)g_sbus_mcre7.rc_channels[5]);
        #endif
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}