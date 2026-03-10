#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

#include "system/system_init.h"
#include "app/blink_task.h"
#include "app/mpu6050.h"
#include "driver/uart.h"

int main(void) {
    system_init();
    uart_send_string("Hello, World!\n");
    xTaskCreate(mpu6050_task, "MPU6050", 512, NULL, configMAX_PRIORITIES - 1, NULL);
    vTaskStartScheduler();
    while(1);
}