#include <FreeRTOS.h>
#include <task.h>

#include "libopencm3/cm3/common.h"
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/usart.h>

#include "driver/uart.h"
#include "dma.h"

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
