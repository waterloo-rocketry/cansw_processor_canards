#include "drivers/adc/adc.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"

extern ADC_HandleTypeDef hadc1;

static SemaphoreHandle_t adc_conversion_semaphore = NULL;
static SemaphoreHandle_t adc_semaphore = NULL;
static adc_channel_t current_adc_channel = PROCESSOR_BOARD_VOLTAGE;

#define ADC_MAX_COUNTS 65535

w_status_t adc_init(void)

{
    adc_conversion_semaphore = xSemaphoreCreateBinary();
    adc_semaphore = xSemaphoreCreateBinary();

    if ((NULL == adc_semaphore) || (NULL == adc_conversion_semaphore)) {
        return W_FAILURE;
    }

    if (xSemaphoreGive(adc_semaphore) != pdTRUE) {
        return W_FAILURE;
    }

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        return W_FAILURE;
    }

    return W_SUCCESS;
}

w_status_t adc_get_value(adc_channel_t channel, uint32_t *output) {
    if (xSemaphoreTake(adc_semaphore, pdMS_TO_TICKS(1)) != pdTRUE) {
        return W_IO_TIMEOUT;
    }

    if (channel != current_adc_channel) {
        ADC_ChannelConfTypeDef sConfig = {0};
        sConfig.Channel = channel;

        // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        //     xSemaphoreGive(adc_semaphore);
        //     return W_FAILURE;
        // }
        current_adc_channel = channel;
    }

    if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
        xSemaphoreGive(adc_semaphore);
        return W_FAILURE;
    }

    if (xSemaphoreTake(adc_conversion_semaphore, pdMS_TO_TICKS(1)) != pdTRUE) {
        HAL_ADC_Stop_IT(&hadc1);
        xSemaphoreGive(adc_semaphore);
        return W_IO_TIMEOUT;
    }

    *output = HAL_ADC_GetValue(&hadc1);

    if (*output > ADC_MAX_COUNTS) {
        HAL_ADC_Stop_IT(&hadc1);
        xSemaphoreGive(adc_semaphore);
        return W_OVERFLOW;
    }

    HAL_ADC_Stop_IT(&hadc1);
    xSemaphoreGive(adc_semaphore);

    return W_SUCCESS;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1) {
    BaseType_t xHigherPriorityTaskWoken;

    /* We have not woken a task at the start of the ISR. */
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(adc_conversion_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
