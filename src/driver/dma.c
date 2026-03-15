#include <FreeRTOS.h>
#include <task.h>

#include "libopencm3/cm3/common.h"
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/usart.h>

#include "driver/uart.h"
#include "dma.h"

char s1[20] = "Hello, DMA World!";
char s2[20] = {0};

/*
void dma1_init() {
    // Enable DMA1 clock
    rcc_periph_clock_enable(RCC_DMA1);

    // DMA mem2mem mode
    dma_enable_mem2mem_mode(DMA1, DMA_CHANNEL1);

    // configure DMA as highest priority
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);

    // 32 bit transfer for source and destination
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_32BIT);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_32BIT);

    // after every 32 bit increment source and destination address
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_enable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);

    // define source as peripheral and destination as memory
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);

    // define source address and destination address
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&s1);
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)&s2);

    // set number of data to transfer before generating interrupt
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, 5);

    // start dma transfer
    dma_enable_channel(DMA1, DMA_CHANNEL1);

    // fucntion to get the interrupt flag
    while (!(DMA_ISR(DMA1) & DMA_ISR_TCIF1));
}
*/

void sbus_dma_init(void) {
    /* clock enable DMA1*/
    rcc_periph_clock_enable(RCC_DMA1);

    /* reset dma*/
    dma_channel_reset(DMA1, DMA_CHANNEL6);

    /* set source/peripheral address*/
    dma_set_peripheral_address(DMA1,
        DMA_CHANNEL6,
        (uint32_t)&USART_DR(USART2));
    
    /* set destination/memory address*/
    dma_set_memory_address(DMA1,
        DMA_CHANNEL6,
        (uint32_t)sbus_dma_buf);

    /* set number of data per transfer after interrupt happen*/
    dma_set_number_of_data(DMA1,
        DMA_CHANNEL6,
        SBUS_DMA_BUF);

    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL6);

    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL6);

    dma_set_peripheral_size(DMA1,
        DMA_CHANNEL6,
        DMA_CCR_PSIZE_8BIT);

    dma_set_memory_size(DMA1,
        DMA_CHANNEL6,
        DMA_CCR_MSIZE_8BIT);

    dma_enable_circular_mode(DMA1, DMA_CHANNEL6);

    dma_enable_channel(DMA1, DMA_CHANNEL6);
    usart_enable_rx_dma(USART2);
}

/*
void dma_task(void *params) {
    (void)params;
    while (1) {
        uart_send_string("Starting DMA transfer...\r\n");
        uart_send_string(s2);
        uart_send_char('\n');
        // dma1_init();
        uart_send_string("After DMA transfer...\r\n");
        uart_send_string(s1);
        uart_send_char('\n');
        uart_send_string(s2);
        uart_send_char('\n');
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
*/