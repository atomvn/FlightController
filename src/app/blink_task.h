#ifndef BLINK_TASK_H
#define BLINK_TASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "../system/system_init.h"

void blink_task(void *pvParameters);
#endif