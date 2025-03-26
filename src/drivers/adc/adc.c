#include "drivers/adc/adc.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define ADC_CONV_TIMEOUT_TICKS pdMS_TO_TICKS(1)

static ADC_HandleTypeDef *adc_handle;
static SemaphoreHandle_t adc_conversion_semaphore = NULL;
static SemaphoreHandle_t adc_mutex = NULL;

static void ADC1_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    (void)hadc;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* We have not woken a task at the start of the ISR. */
    xSemaphoreGiveFromISR(adc_conversion_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

w_status_t adc_init(ADC_HandleTypeDef *hadc) {
    if (NULL == hadc) {
        return W_INVALID_PARAM;
    }
    adc_handle = hadc;

    adc_conversion_semaphore = xSemaphoreCreateBinary();
    adc_mutex = xSemaphoreCreateMutex();

    if ((NULL == adc_mutex) || (NULL == adc_conversion_semaphore)) {
        return W_FAILURE;
    }

    if (HAL_OK != HAL_ADC_RegisterCallback(
                      adc_handle, HAL_ADC_CONVERSION_COMPLETE_CB_ID, ADC1_ConvCpltCallback
                  )) {
        return W_FAILURE;
    }

    // Run ADC hardware auto-calibration
    if (HAL_OK != HAL_ADCEx_Calibration_Start(adc_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)) {
        return W_FAILURE;
    }

    return W_SUCCESS;
}

w_status_t adc_get_value(adc_channel_t channel, uint32_t *output, uint32_t timeout_ms) {
    // Currently, there is only one channel which we read from, and our channel enum
    // does not map directly to that channel, so handling multiple channels on the
    // same ADC would require a lot more logic. For now, return an error if something tries to
    // call a different adc channel
    if (PROCESSOR_BOARD_VOLTAGE != channel) {
        return W_INVALID_PARAM;
    }

    if (pdTRUE != xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(timeout_ms))) {
        return W_FAILURE;
    }

    if (HAL_OK != HAL_ADC_Start_IT(adc_handle)) {
        xSemaphoreGive(adc_mutex);
        return W_IO_TIMEOUT;
    }

    if (pdTRUE != xSemaphoreTake(adc_conversion_semaphore, ADC_CONV_TIMEOUT_TICKS)) {
        HAL_ADC_Stop_IT(adc_handle);
        xSemaphoreGive(adc_mutex);
        return W_IO_TIMEOUT;
    }

    *output = HAL_ADC_GetValue(adc_handle);

    if (*output > ADC_MAX_COUNTS) {
        // HAL_ADC_Stop_IT(adc_handle);
        xSemaphoreGive(adc_mutex);
        // TODO: log error?
        return W_OVERFLOW;
    }

    // HAL_ADC_Stop_IT(adc_handle); // We should not need to stop the ADC since it is not in
    // continuous mode
    xSemaphoreGive(adc_mutex);

    return W_SUCCESS;
}

