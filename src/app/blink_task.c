#include "blink_task.h"

void blink_task(void *pvParameters) {
    while (1) {
        gpio_toggle(GPIOC, GPIO13);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}