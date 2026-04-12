#ifndef MCRE7_V2_H
#define MCRE7_V2_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "driver/dma.h"

#define CHANNEL_NUM 16

/*********************** MCRE7 V2 State ***********************/
typedef int32_t mcre7_v2_error_t;
#define MCRE7_V2_OK             0
#define MCRE7_V2_DMA_ERROR    (-1)
#define MCRE7_V2_MUTEX_ERROR  (-2)
#define MCRE7_V2_INVALID_ARG  (-3)

/* S-BUS frame structure */
typedef struct {
    uint16_t rc_channels[CHANNEL_NUM];
    SemaphoreHandle_t mutex;
} sbus_mcre7_t;

/* Global instance of S-BUS MCRE7 state */
extern sbus_mcre7_t g_sbus_mcre7;

/* ======================== Public API ======================== */
mcre7_v2_error_t mcre7_v2_init(void);
mcre7_v2_error_t read_mcre7_v2_channels(uint16_t* channels, uint32_t timeout);
void mcre7_v2_task(void* params);

#endif // MCRE7_V2_H