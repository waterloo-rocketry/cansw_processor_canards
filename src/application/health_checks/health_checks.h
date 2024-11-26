#ifndef HEALTH_CHECKS_H
#define HEALTH_CHECKS_H

#include <stdbool.h>
#include <stdint.h>

#define ADC1_MAX_COUNTS 65535.0f
#define ADC1_VREF 3.3f
#define R_SENSE 0.033f
#define INA180A3_GAIN 100.0f

static inline float adc1_voltage_v(uint32_t counts) {
    return (((float)(counts)) / ADC1_MAX_COUNTS) * ADC1_VREF;
}

static inline float adc1_curr_ma(float voltage) {
    return (voltage / INA180A3_GAIN / R_SENSE);
}

#define MAX_CURR_5V_mA 400

#define NUM_WATCHDOG_TASKS 12

w_status_t health_check_init(void);
void health_check_task(void *argument);
void watchdog_kick(uint32_t task_id);

#endif
