#include "hal_adc_mock.h"

// Define the mock functions - exact match with declarations
DEFINE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_ADC_RegisterCallback, ADC_HandleTypeDef *, HAL_ADC_CallbackIDTypeDef,
    pADC_CallbackTypeDef
);
DEFINE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_ADCEx_Calibration_Start, ADC_HandleTypeDef *, uint32_t, uint32_t
);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_ADC_Start_IT, ADC_HandleTypeDef *);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_ADC_Stop_IT, ADC_HandleTypeDef *);
DEFINE_FAKE_VALUE_FUNC(uint32_t, HAL_ADC_GetValue, ADC_HandleTypeDef *);
