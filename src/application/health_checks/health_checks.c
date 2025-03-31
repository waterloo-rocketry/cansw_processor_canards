#include "health_checks.h"
#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "drivers/adc/adc.h"
#include "drivers/timer/timer.h"
#include "printf.h"
#include "task.h"
#include "message_types.h"

#define TASK_DELAY_MS 1000
#define ADC_VREF 3.3f
#define R_SENSE 0.033f
#define INA180A3_GAIN 100.0f
#define MAX_CURRENT_mA 400
#define MAX_WATCHDOG_TASKS 10

// struct for watchdog
typedef struct {
    TaskHandle_t task_handle;
    bool is_kicked; 
    float last_kick_timestamp;
    uint32_t timeout_ticks;
} watchdog_task_t;



//watchdog initiailsations
static watchdog_task_t watchdog_tasks[MAX_WATCHDOG_TASKS];
static uint32_t num_watchdog_tasks = 0;



w_status_t get_adc_current(uint32_t *adc_current_mA) {
    w_status_t status = W_SUCCESS;
    uint32_t adc_value;

    status |= adc_get_value(PROCESSOR_BOARD_VOLTAGE, &adc_value, TASK_DELAY_MS);
    if (status != W_SUCCESS) {
        return status;
    }

    uint32_t voltage_mV = ((float)(adc_value) / ADC_MAX_COUNTS) * ADC_VREF *
                          1000.0f; // TODO this loses precision for no good reason ??
    *adc_current_mA = ((float)(voltage_mV) / INA180A3_GAIN) / R_SENSE;

    return W_SUCCESS;
}



w_status_t check_current(void){
    uint32_t adc_current_mA;
    w_status_t status = W_SUCCESS;

    status |= get_adc_current(&adc_current_mA);
    if (status == W_SUCCESS) {
        float ms = 0;
        timer_get_ms(&ms);
        can_msg_t msg = {0};

        if (adc_current_mA > MAX_CURRENT_mA) {
            if (false == build_general_board_status_msg(PRIO_HIGH, (uint16_t)ms, E_5V_OVER_CURRENT, adc_current_mA, &msg)) {
                log_text("health_checks", "E_5V_OVER_CURRENT board status error");
                return W_FAILURE;
            }
        } 
        else {
            if (false == build_general_board_status_msg(PRIO_LOW, (uint16_t)ms, E_NOMINAL, adc_current_mA, &msg)) {
                log_text("health_checks", "E_NOMINAL board status error");
                return W_FAILURE;
            }
        }
            status |= can_handler_transmit(&msg);
        }

    return status; 
}



w_status_t health_check_init(void) {

    for (uint32_t i = 0; i < MAX_WATCHDOG_TASKS; i++) {
        watchdog_tasks[i].task_handle = NULL;
        watchdog_tasks[i].is_kicked = false;
        watchdog_tasks[i].last_kick_timestamp = 0.0f;
        watchdog_tasks[i].timeout_ticks = 0;
    }
    num_watchdog_tasks = 0;

    return W_SUCCESS;
}





w_status_t watchdog_kick(void) {
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    float current_time = 0.0f;
    w_status_t status = W_SUCCESS;

    status |= timer_get_ms(&current_time);
    if (status != W_SUCCESS) {
        log_text("health_checks", "timer_get_ms failure");
        return status;
    }

    // Check if the current task is registered
    for (uint32_t i = 0; i < num_watchdog_tasks; i++) {
        if (watchdog_tasks[i].task_handle == current_task) {
            watchdog_tasks[i].is_kicked = true;
            watchdog_tasks[i].last_kick_timestamp = current_time;
            return W_SUCCESS;
        }
    }
    return W_FAILURE;
}





w_status_t watchdog_register_task(TaskHandle_t task_handle, uint32_t timeout_ticks) {
    if (task_handle == NULL || timeout_ticks == 0) {
        log_text("health_checks", "invalid arguments into watchdog register");
        return W_FAILURE;
    }

    if (num_watchdog_tasks >= MAX_WATCHDOG_TASKS) {
        log_text("health_checks", "max watchdog tasks reached");
        return W_FAILURE;
    }

    // Check if the task is already registered
    for (uint32_t i = 0; i < num_watchdog_tasks; i++) {
        if (watchdog_tasks[i].task_handle == task_handle) {
        log_text("health_checks", "duplicate task registration:%p", (void*)task_handle);
        return W_FAILURE;
        }
    }

    float current_time = 0.0f;
    w_status_t status = W_SUCCESS;
    status |= timer_get_ms(&current_time);

    watchdog_tasks[num_watchdog_tasks].task_handle = task_handle;
    watchdog_tasks[num_watchdog_tasks].is_kicked = true;
    watchdog_tasks[num_watchdog_tasks].last_kick_timestamp = current_time;
    watchdog_tasks[num_watchdog_tasks].timeout_ticks = timeout_ticks;

    num_watchdog_tasks++; //incriminent the watchdog task count for future ref

    return status;
}




w_status_t check_watchdog_tasks(void) {
    w_status_t status = W_SUCCESS;
    float current_time = 0.0f;
    can_msg_t msg = {0};

    status |= timer_get_ms(&current_time);
    if (status != W_SUCCESS) {
        log_text("health_checks", "timer_get_ms failure");
        return status;
    }

    for (uint32_t i = 0; i < num_watchdog_tasks; i++) {
        if (watchdog_tasks[i].task_handle != NULL) {
            float time_elapsed = current_time - watchdog_tasks[i].last_kick_timestamp;
            uint32_t ticks_elapsed = pdMS_TO_TICKS((uint32_t)time_elapsed);  //time to ticks

            if (watchdog_tasks[i].is_kicked || (ticks_elapsed <= watchdog_tasks[i].timeout_ticks)) {
                //do nothing if any one is true
                    }
            else{
                    if (false == build_general_board_status_msg(PRIO_HIGH, (uint16_t)current_time, E_WATCHDOG_TIMEOUT, i, &msg)){ 
                        log_text("health_checks", "E_WATCHDOG_TIMEOUT status message error");
                        return W_FAILURE;
                    }

                status |= can_handler_transmit(&msg);

            }
            //resetting for next check
            watchdog_tasks[i].is_kicked = false;
        }
    }
    return status;
}




w_status_t health_check_exec() {
    w_status_t status = W_SUCCESS;

    status |= check_current();
    status |= check_watchdog_tasks();

    return status;
}




void health_check_task() {
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {
        health_check_exec();
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TASK_DELAY_MS));
        }
    }
