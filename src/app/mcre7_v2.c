#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/gpio.h>

#include "mcre7_v2.h"
#include "driver/uart.h"

void sbus_parse(void) {
    for(int i=0; i<SBUS_DMA_BUF-25; i++) {
        if (sbus_dma_buf[i] == 0x0F && sbus_dma_buf[i+24] == 0x00) {
            sbus_decode(&sbus_dma_buf[i]);
        }
    }
}

void sbus_decode(volatile uint8_t* buf) {
    rc_channels[0]  = (buf[1]  | buf[2] << 8) & 0x07FF;
    rc_channels[1]  = (buf[2] >> 3 | buf[3] << 5) & 0x07FF;
    rc_channels[2]  = (buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF;
    rc_channels[3]  = (buf[5] >> 1 | buf[6] << 7) & 0x07FF;
    rc_channels[4]  = (buf[6] >> 4 | buf[7] << 4) & 0x07FF;
    rc_channels[5]  = (buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF;
    rc_channels[6]  = (buf[9] >> 2 | buf[10] << 6) & 0x07FF;
    rc_channels[7]  = (buf[10] >> 5 | buf[11] << 3) & 0x07FF;
}

void mcre7_v2_task(void* params){
    (void)params;
    sbus_dma_init();
    while(1) {
        sbus_parse();
        // uart_send_string("***********************MCRE data*********************\n");
        // uart_printf("Channel 1: %u\n", (uint32_t)rc_channels[0]);
        // uart_printf("Channel 2: %u\n", (uint32_t)rc_channels[1]);
        // uart_printf("Channel 3: %u\n", (uint32_t)rc_channels[2]);
        // uart_printf("Channel 4: %u\n", (uint32_t)rc_channels[3]);
        // uart_printf("Channel 5: %u\n", (uint32_t)rc_channels[4]);
        // uart_printf("Channel 6: %u\n", (uint32_t)rc_channels[5]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}