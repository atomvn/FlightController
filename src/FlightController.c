#include "system/system_init.h"
#include "app/blink_task.h"

int main(void) {
    system_init();
    xTaskCreate(blink_task, "Blink", 512, NULL, configMAX_PRIORITIES - 1, NULL);
    vTaskStartScheduler();
    while(1);
}