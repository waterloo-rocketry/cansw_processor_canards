#include "health_checks.h"
#include "FreeRTOS.h"
#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "drivers/adc/adc.h"
#include "drivers/timer/timer.h"
#include "printf.h"
#include "task.h"

#define TASK_DELAY_MS 1000

#define ADC_MAX_COUNTS 65535.0f
#define ADC_VREF 3.3f
#define R_SENSE 0.033f
#define INA180A3_GAIN 100.0f
#define MAX_CURRENT_mA 400

static w_status_t get_adc_current(uint32_t *adc_current_mA) {
    w_status_t status = W_SUCCESS;
    uint32_t adc_value;
    status |= adc_get_value(PROCESSOR_BOARD_VOLTAGE, &adc_value);
    // TODO: log and return result of adc value retrieval success/fail

    uint32_t voltage_mV = ((float)(adc_value) / ADC_MAX_COUNTS) * ADC_VREF *
                          1000.0f; // TODO this loses precision for no good reason
    *adc_current_mA = ((float)(voltage_mV) / INA180A3_GAIN) / R_SENSE;

    return W_SUCCESS;
}

w_status_t health_check_init(void) {
    // TODO: initialize watchdog tasks
    return W_SUCCESS;
}

// TODO: void watchdog_kick(void)

// TODO: void watchdog_register_task(TaskHandle_t task_handle)

w_status_t health_check_exec() {
    w_status_t status = W_SUCCESS;
    uint32_t adc_current_mA;

    if (W_SUCCESS == get_adc_current(&adc_current_mA)) {
        float ms = 0;
        timer_get_ms(&ms);
        can_msg_t msg = {0};
        if (adc_current_mA > MAX_CURRENT_mA) {
            if (false == build_general_board_status_msg(
                PRIO_HIGH, (uint16_t)ms, E_5V_OVER_CURRENT, adc_current_mA, &msg
            )) {
                return W_FAILURE;
            }
        } else {
            if (false == build_general_board_status_msg(
                PRIO_LOW, (uint16_t)ms, E_NOMINAL, adc_current_mA, &msg
            )) {
                return W_FAILURE;
            }
        }

        status |= can_handler_transmit(&msg);
        // TODO: monitor watchdog tasks
    }

    return status;
}

void health_check_task(void *argument) {
    (void)argument;
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {
        health_check_exec();

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TASK_DELAY_MS));
        // TODO: check time doesn't overrun past 1000ms
    }
}
