#include "FreeRTOS.h"
#include "printf.h"

#include "health_checks.h"
#include "log.h"
#include "can_handler.h
#include "adc.h"


#define NUM_WATCHDOG_TASKS 2
#define TASK1_TIMEOUT_MS 1000
#define TASK2_TIMEOUT_MS 2000

typedef struct {
    bool is_kicked;
    TickType_t last_kick_time;
    TickType_t timeout_ticks;
} WatchdogTask;


static WatchdogTask watchdog_tasks[NUM_WATCHDOG_TASKS];

extern uint16_t ADC_get_board_current(void);


w_status_t health_check_init(void) {
    for (uint8_t i = 0; i < NUM_WATCHDOG_TASKS; i++) {
        watchdog_tasks[i].is_kicked = false;
        watchdog_tasks[i].last_kick_time = xTaskGetTickCount();
        watchdog_tasks[i].timeout_ticks = (i == 0) ? pdMS_TO_TICKS(TASK1_TIMEOUT_MS) : pdMS_TO_TICKS(TASK2_TIMEOUT_MS);
    }
    logInfo("Watchdog", "Initialized with %d tasks", NUM_WATCHDOG_TASKS);
}


void watchdog_kick(uint8_t task_id) {
    if (task_id < NUM_WATCHDOG_TASKS) {
        watchdog_tasks[task_id].is_kicked = true;
        watchdog_tasks[task_id].last_kick_time = xTaskGetTickCount();
    }
}


void health_check_task(void *argument) {
    TickType_t lastWakeTime = xTaskGetTickCount(); // For periodic task execution

    for (;;) {
        // Part 1: Monitor ADC current
        uint16_t adc_current_mA = ADC_GetCurrent_mA();

        if (adc_current_mA > MAX_CURRENT_mA) {
            // Send overcurrent CAN message
            can_msg_t msg;
            uint8_t current_data[2] = {
                (adc_current_mA >> 8) & 0xFF, // High byte
                adc_current_mA & 0xFF        // Low byte
            };
            build_board_stat_msg(millis_(), E_5V_OVER_CURRENT, current_data, 2, &msg);
            xQueueSend(busQueue, &msg, 10);

            // Log overcurrent error
            logError("HealthCheck", "Overcurrent detected: %dmA", adc_current_mA);
        } else if (adc_current_mA < MIN_CURRENT_mA) {
            logError("HealthCheck", "Undercurrent detected: %dmA", adc_current_mA);
        } else {
            // Nominal current status
            can_msg_t msg;

            // Send nominal status message
            build_board_stat_msg(millis_(), E_NOMINAL, NULL, 0, &msg);
            xQueueSend(busQueue, &msg, 10);

            // Send current draw message via CAN
            build_analog_data_msg(millis_(), SENSOR_5V_CURR, adc_current_mA, &msg);
            xQueueSend(busQueue, &msg, 10);

            // Log nominal status
            logInfo("HealthCheck", "Current draw: %dmA", adc_current_mA);
        }

        // Part 2: Monitor watchdog kicks
        TickType_t current_time = xTaskGetTickCount();
        for (uint8_t i = 0; i < NUM_WATCHDOG_TASKS; i++) {
            if (watchdog_tasks[i].is_kicked) {
                // Reset the kick flag for the next iteration
                watchdog_tasks[i].is_kicked = false;
            } else if ((current_time - watchdog_tasks[i].last_kick_time) > watchdog_tasks[i].timeout_ticks) {
                // Task missed its watchdog kick
                logCritical("Watchdog", "Task %d missed its kick!", i);

                // Trigger additional error handling if needed
                logError("HealthCheck", "Critical issue detected for Task %d!", i);
            }
        }

        // Task delay to run periodically (e.g., every 1 second)
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000));
    }
}
