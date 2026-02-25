#include "systick.h"

static volatile uint32_t system_millis = 0;

/* Initialize SysTick to 1ms tick */
void systick_init(uint32_t sysclk_hz)
{
    uint32_t reload = (sysclk_hz / 1000) - 1;

    STK_CSR = 0;              // Disable SysTick
    STK_RVR = reload;         // Set reload value
    STK_CVR = 0;              // Clear current value
    STK_CSR = STK_CSR_CLKSOURCE |
              STK_CSR_TICKINT  |
              STK_CSR_ENABLE;  // Enable SysTick + IRQ
}

// Define systick handle
void sys_tick_handler(void) {
    system_millis++;
}

// Millis function to return current time
uint32_t millis(void) {
    return system_millis;
}