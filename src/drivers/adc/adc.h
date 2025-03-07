#ifndef ADC_H
#define ADC_H

#include "rocketlib/include/common.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define ADC_MAX_COUNTS 0xFFFF // 16 bit full scale, configured in ioc

typedef enum {
    PROCESSOR_BOARD_VOLTAGE = 0,
    ADC_CHANNEL_COUNT
} adc_channel_t;

/**
 * @brief Initialize the ADC driver
 * @param hadc Pointer to the HAL ADC handle
 * @return Status of the initialization
 */
w_status_t adc_init(ADC_HandleTypeDef *hadc);

/**
 * @brief Get the raw (16 bit) value of the specified ADC channel.
 * @param channel The adc channel to read from
 * @param output Pointer to store output value of the ADC channel
 * @param timeout_ms How long to wait to acquire the mutex. Conversion timeout is fixed at 1ms in
 * addition to this
 * @return Status of the read operation
 */
w_status_t adc_get_value(adc_channel_t channel, uint32_t *output, uint32_t timeout_ms);

#endif
