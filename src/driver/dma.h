#ifndef DMA_H
#define DMA_H

#define SBUS_DMA_BUF 64
volatile uint8_t sbus_dma_buf[SBUS_DMA_BUF]; 
void sbus_dma_init(void);

#endif