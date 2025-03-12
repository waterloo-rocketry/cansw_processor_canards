#include "health_checks.h"
#include "FreeRTOS.h"
#include "drivers/adc/adc.h"
#include "application/can_handler/can_handler.h"
#include "application/logger/log.h"
#include "printf.h"

static w_status_t get_adc_current(uint16_t *adc_current_mA) {
  uint32_t adc_value;
  w_status_t status = adc_get_value(PROCESSOR_BOARD_VOLTAGE, &adc_value);

  if (status != W_SUCCESS) {
    // TODO: Log status for successful adc value retrieval
    return status;
  }

  // TODO: get ADC values adc_get_constants();

  if (adc_value > constants.ADC_MAX_COUNTS) {
    log_text("HealthCheck", "ADC value overflow: %lu", adc_value);
    return W_OVERFLOW;
  }

  uint16_t voltage_mV =
      (uint16_t)((((float)adc_value) / constants.ADC_MAX_COUNTS) *
                 constants.ADC_VREF) *
      1000;
  *adc_current_mA =
      (uint16_t)((voltage_mV / constants.INA180A3_GAIN) / constants.R_SENSE);

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
      if (adc_current_mA > constants.MAX_CURRENT_mA) {
        can_msg_t msg;
        uint8_t current_data[2] = {
            (adc_current_mA >> 8) & 0xFF, // High byte
            adc_current_mA & 0xFF         // Low byte
        };

        // TODO: build_msg()
        // TODO: send CAN message
        if (can_handle_tx(&msg) != W_SUCCESS) {
          log_text("HealthCheck", "Failed to send CAN message");
        }

        log_text("HealthCheck", "Overcurrent detected: %dmA", adc_current_mA);
      } else if (adc_current_mA < constants.MIN_CURRENT_mA) {
        log_text("HealthCheck", "Undercurrent detected: %dmA", adc_current_mA);
      } else {
        log_text("HealthCheck", "Current draw nominal: %dmA", adc_current_mA);
      }

      // TODO: monitor watchdog tasks
    }
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000));
  }
}
