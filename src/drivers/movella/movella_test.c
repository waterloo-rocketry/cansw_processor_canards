#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "movella.h"
#include "../uart/uart.h"

#define MOVELLA_TEST_TASK_PRIORITY   (3)
#define MOVELLA_TEST_STACK_SIZE      (1024)
#define MOVELLA_TEST_PERIOD_MS       (100)

#ifndef portTICK_PERIOD_MS
#define portTICK_PERIOD_MS (1000 / configTICK_RATE_HZ)
#endif

static void movella_test_task(void *parameters);

w_status_t movella_test_init(void);

w_status_t movella_test_init(void) {
    BaseType_t task_created = xTaskCreate(
        movella_test_task,
        "MovellaTest",
        MOVELLA_TEST_STACK_SIZE,
        NULL,
        MOVELLA_TEST_TASK_PRIORITY,
        NULL
    );

    return (task_created == pdPASS) ? W_SUCCESS : W_FAILURE;
}

static void print_movella_data(const movella_data_t *data) {
    printf("Movella Sensor Data:\n");
    printf("  Acceleration (m/s^2): X=%.2f, Y=%.2f, Z=%.2f\n", 
           data->acc.component.x, data->acc.component.y, data->acc.component.z);
    printf("  Gyroscope (rad/s): X=%.2f, Y=%.2f, Z=%.2f\n", 
           data->gyr.component.x, data->gyr.component.y, data->gyr.component.z);
    printf("  Euler Angles (deg): Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", 
           data->euler.component.x, data->euler.component.y, data->euler.component.z);
    printf("  Magnetometer: X=%.2f, Y=%.2f, Z=%.2f\n", 
           data->mag.component.x, data->mag.component.y, data->mag.component.z);
    printf("  Pressure: %.2f Pa\n", data->pres);
    printf("  Temperature: %.2f °C\n", data->temp);
    printf("\n");
}

static void movella_test_task(void *parameters) {
    w_status_t status;
    movella_data_t sensor_data;
    uint32_t start_tick, elapsed_ms;
    
    printf("Initializing Movella driver...\n");

    status = movella_init();
    if (status != W_SUCCESS) {
        printf("Failed to initialize Movella driver! Error code: %d\n", status);
        vTaskDelete(NULL);
        return;
    }
    
    printf("Movella driver initialized successfully\n");
    printf("Waiting for data...\n\n");
    
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        start_tick = xTaskGetTickCount();
        
        status = movella_get_data(&sensor_data);
        if (status == W_SUCCESS) {
            print_movella_data(&sensor_data);
        } else {
            printf("Error reading Movella data: %d\n", status);
        }
        
        vTaskDelay(pdMS_TO_TICKS(MOVELLA_TEST_PERIOD_MS));
    }

    vTaskDelete(NULL);
}
