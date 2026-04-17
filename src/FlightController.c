#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

#include "driver/uart.h"
#include "driver/dma.h"
#include "system/system_init.h"
#include "app/blink_task.h"
#include "app/mpu6050.h"
#include "app/mcre7_v2.h"
#include "app/flight_controller.h"

int main(void) {
    system_init();
    uart_send_string("Hello, World!\n");
    xTaskCreate(blink_task, "Blink task", 512, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(mpu6050_task, "MPU6050 task", 512, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(mcre7_v2_task, "MCRE7_v2 task", 512, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(flight_task, "Flight task", 512, NULL, configMAX_PRIORITIES - 2, NULL);
    vTaskStartScheduler();
    while(1);
}