#ifndef MCRE7_V2_H
#define MCRE7_V2_H
#include <stdint.h>

#include "driver/dma.h"

uint16_t rc_channels[16];
void sbus_parse(void);
void sbus_decode(volatile uint8_t* buf);
void mcre7_v2_task(void* params);

#endif