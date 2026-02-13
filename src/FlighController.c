#include <stdio.h>
#include "drivers/rcc/rcc.h"
#include "drivers/gpio/gpio.h"

int main(void) {
    clock_setup_pll_72mhz();
    gpio_enable_clock(GPIOC);
    gpio_mode_setup(GPIOC, 13, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL);
    gpio_set(GPIOC, 13);

    while(1) {
        gpio_toggle(GPIOC, 13);
        delay_ms(72000000/4);
    }
}