#ifndef DMA_H
#define DMA_H

#include <stdint.h>
#include <stdbool.h>

/* ======================== Configuration ======================== */
/** @brief Size of the SBUS DMA buffer */
#define SBUS_DMA_BUF_SIZE       64
#define GPS_DMA_BUF_SIZE        256

/** @brief S-BUS DMA controller and channel configuration */
#define SBUS_DMA_CONTROLLER     DMA1
#define SBUS_DMA_CHANNEL        DMA_CHANNEL6
#define SBUS_USART_NUM          2
#define SBUS_USART_DR           USART_DR(USART2)
#define GPS_DMA_CONTROLLER      DMA1
#define GPS_DMA_CHANNEL         DMA_CHANNEL3
#define GPS_USART_NUM           3
#define GPS_USART_DR            USART_DR(USART3)

/*********************** S-BUS DMA State ***********************/
typedef int32_t dma_error_t;
#define DMA_OK                      0
#define DMA_INITIALIZED           (-1)
#define DMA_UART_NOT_ENABLED      (-2)
#define DMA_NOT_INITIALIZED       (-3)

/* ======================== DMA Buffer Management ======================== */
/** @brief S-BUS DMA state structure */
typedef struct {
    uint8_t sbus_dma_buf[SBUS_DMA_BUF_SIZE]; 
    bool frame_ready;
    bool initialized;
} sbus_dma_state_t;

/** @brief GPS DMA state structure */
typedef struct {
    char gps_dma_buf[GPS_DMA_BUF_SIZE];
    bool frame_ready;
    bool initialized;
} gps_dma_state_t;

/** @brief Global instance of S-BUS DMA state */
extern sbus_dma_state_t g_sbus_dma_state;
/** @brief Global instance of GPS DMA state */
extern gps_dma_state_t g_gps_dma_state;

/* ======================== Public API ======================== */
dma_error_t sbus_dma_init(void);

dma_error_t sbus_dma_deinit(void);

dma_error_t gps_dma_init(void);
#endif // DMA_H