#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "drivers/rcc/rcc.h"
#include "drivers/gpio/gpio.h"
#include "drivers/systick/systick.h"

void task(void *para) {
    gpio_toggle(GPIOC, 13);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

int main(void) {
    clock_setup_pll_72mhz();
    systick_init(72000000);
    BaseType_t ret = xTaskCreate(task, "task", 1024, NULL, 1, NULL);
    vTaskStartScheduler();
    gpio_enable_clock(GPIOC);
    gpio_mode_setup(GPIOC, 13, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL);
    gpio_set(GPIOC, 13);

}