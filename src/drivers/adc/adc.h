#ifndef ADC_H
#define ADC_H

#include <stdbool.h>

typedef enum {
    PROCESSOR_BOARD_VOLTAGE = 0,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4,
    ADC_CHANNEL_5
} adc_channel_t;

w_status_t adc_init(void);

// writes adc value of the specified channel to the output parameter
w_status_t adc_get_value(adc_channel_t channel, uint32_t *output);

// writes processor board voltage to the output parameter
w_status_t adc_get_board_voltage(uint16_t *output);

// writes processor board voltage to the output parameter
w_status_t adc_get_board_current(uint16_t *output);

#endif
