/*
 * health_checks.h
 *
 * Created on: Nov 24, 2024
 *
 */

#ifndef HEALTH_CHECKS_H_
#define HEALTH_CHECKS_H

#include <stdbool.h>
#include <stdint.h>

#define ADC1_MAX_COUNTS 65535.0
#define ADC1_VREF 3.3
#define R_SENSE 0.033
#define INA180A3_GAIN 100.0

#define ADC1_VOLTAGE_V(counts)(counts / ADC1_MAX_COUNTS * ADC1_VREF)
#define ADC1_CURR_mA(voltage) (voltage / INA180A3 / R_SENSE )

#define MAX_CURR_5V_mA 400

#define NUM_WATCHDOG_TASKS 12

bool health_check_init();
void health_check_task(void *argument);
void watchdog_kick(uint32_t task_id);

#endif
