#ifndef ADC_H
#define ADC_H

#include "rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PROCESSOR_BOARD_VOLTAGE = 0,
    ADC_CHANNEL_COUNT
} adc_channel_t;

w_status_t adc_init(void);

// writes adc value of the specified channel to the output parameter
// blocks until adc conversion is finished, or times out after 1 ms
// is thread safe: semaphores prevent multiple conversions from happening at once
w_status_t adc_get_value(adc_channel_t channel, uint32_t *output);

#endif
