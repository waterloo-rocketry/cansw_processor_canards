#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "application/controller/controller.h"

// temporary empty example for the freertos sim testing
void controller_task(void *argument) {
    while (true) {
        printf("running controller task! %d\n", xTaskGetTickCount());
        vTaskDelay(1000);
    }
}
