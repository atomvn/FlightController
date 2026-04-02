#include "FreeRTOS.h"
#include "task.h"
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/f1/gpio.h>

#include "blink_task.h"

void blink_task(void *pvParameters) {
    while (1) {
        gpio_toggle(GPIOC, GPIO13);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}