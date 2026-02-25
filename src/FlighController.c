#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "drivers/rcc/rcc.h"
#include "drivers/gpio/gpio.h"
#include "drivers/systick/systick.h"
#include "drivers/uart/uart.h"

void task(void *para) {
    while(1) {
        gpio_toggle(GPIOC, 13);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

int main(void) {
    clock_setup_pll_72mhz();
    systick_init(72000000);
    uart1_init();
    gpio_enable_clock(GPIOC);
    gpio_mode_setup(GPIOC, 13, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL);
    gpio_set(GPIOC, 13);
    BaseType_t ret = xTaskCreate(task, "task", 512, NULL, 1, NULL);
    if (ret != pdPASS) {
        uart1_send_string("Task creation failed!\n");
        while(1);
    }
    else {
        uart1_send_string("Task created successfully!\n");
    }
    vTaskStartScheduler();
    // while(1) {
    //     gpio_toggle(GPIOC, 13);
    //     delay_ms(72000000/6);
    //     uart1_send_string("Hello, World!\n");
    // }
}