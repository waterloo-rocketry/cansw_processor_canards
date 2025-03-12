#include "FreeRTOS.h"
#include "task.h"
#include "health_checks.h"
#include "drivers/adc/adc.h"
#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "printf.h"

#define ADC_MAX_COUNTS 65535.0
#define ADC_VREF 3.3
#define R_SENSE 0.033
#define INA180A3_GAIN 100.0
#define MAX_CURRENT_mA 400

static w_status_t get_adc_current(uint16_t *adc_current_mA) {
  uint32_t adc_value;
  // TODO: get ADC values from adc_get_value()
  // TODO: log and return result of adc value retrieval success/fail
 
  // TODO: get ADC constants from adc_get_constants();
  // TODO: compare ADC value with ADC constants, log and return result

  uint16_t voltage_mV =
      (uint16_t)((((float)adc_value) / ADC_MAX_COUNTS) *
                 ADC_VREF) *
      1000;
  *adc_current_mA =
      (uint16_t)((voltage_mV / INA180A3_GAIN) / R_SENSE);

  return W_SUCCESS;
}

w_status_t health_check_init(void) {
  // TODO: initialize watchdog tasks
  return W_SUCCESS;
}

// TODO: void watchdog_kick(void)

// TODO: void watchdog_register_task(TaskHandle_t task_handle)

void health_check_task(void *argument) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  uint16_t adc_current_mA;

  // TODO: get ADC values adc_constants_t constants = adc_get_constants();

  for (;;) {
    if (get_adc_current(&adc_current_mA) == W_SUCCESS) {
      if (adc_current_mA > MAX_CURRENT_mA) {
        can_msg_t msg;
        uint8_t current_data[2] = {
            (adc_current_mA >> 8) & 0xFF, // High byte
            adc_current_mA & 0xFF         // Low byte
        };

        // TODO: build_msg()
        // TODO: send CAN message 
        if (can_handle_tx(&msg) != W_SUCCESS) {
          // TODO: log if sent successfully
        }

        // TODO: log if overcurrent detected
      } else {
        //TODO: log current draw
      }

      // TODO: monitor watchdog tasks
    }
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000));
    // TODO: check time doesn't overrun past 1000ms
  }
}
