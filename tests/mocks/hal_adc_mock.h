#ifndef MOCK_ADC_HAL_H
#define MOCK_ADC_HAL_H

#include "fff.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

#define ADC_CALIB_OFFSET 0x0
#define ADC_CALIB_OFFSET_LINEARITY 0x1
#define ADC_SINGLE_ENDED 0x0
#define ADC_DIFFERENTIAL_ENDED 0x1

typedef struct __ADC_HandleTypeDef {
    uint32_t *Instance; /*!< Register base address */
    uint32_t Init; /*!< ADC initialization parameters and regular conversions setting */
    uint32_t *DMA_Handle; /*!< Pointer DMA Handler */
    uint32_t Lock; /*!< ADC locking object */
    uint32_t State; /*!< ADC communication state (bitmap of ADC states) */
    uint32_t ErrorCode; /*!< ADC Error code */
    uint32_t InjectionConfig; /*!< ADC injected channel configuration build-up structure */
    void (*ConvCpltCallback)(struct __ADC_HandleTypeDef *hadc
    ); /*!< ADC conversion complete callback */
    void (*ConvHalfCpltCallback)(struct __ADC_HandleTypeDef *hadc
    ); /*!< ADC conversion DMA half-transfer callback */
    void (*LevelOutOfWindowCallback)(struct __ADC_HandleTypeDef *hadc
    ); /*!< ADC analog watchdog 1 callback */
    void (*ErrorCallback)(struct __ADC_HandleTypeDef *hadc); /*!< ADC error callback */
    void (*InjectedConvCpltCallback)(struct __ADC_HandleTypeDef *hadc
    ); /*!< ADC group injected conversion complete callback */
    void (*InjectedQueueOverflowCallback)(struct __ADC_HandleTypeDef *hadc
    ); /*!< ADC group injected context queue overflow callback */
    void (*LevelOutOfWindow2Callback)(struct __ADC_HandleTypeDef *hadc
    ); /*!< ADC analog watchdog 2 callback */
    void (*LevelOutOfWindow3Callback)(struct __ADC_HandleTypeDef *hadc
    ); /*!< ADC analog watchdog 3 callback */
    void (*EndOfSamplingCallback)(struct __ADC_HandleTypeDef *hadc
    ); /*!< ADC end of sampling callback */
    void (*MspInitCallback)(struct __ADC_HandleTypeDef *hadc); /*!< ADC Msp Init callback */
    void (*MspDeInitCallback)(struct __ADC_HandleTypeDef *hadc); /*!< ADC Msp DeInit callback */
} ADC_HandleTypeDef;

/**
 * @brief  HAL ADC Callback ID enumeration definition
 */
typedef enum {
    HAL_ADC_CONVERSION_COMPLETE_CB_ID = 0x00U, /*!< ADC conversion complete callback ID */
    HAL_ADC_CONVERSION_HALF_CB_ID = 0x01U, /*!< ADC conversion DMA half-transfer callback ID */
    HAL_ADC_LEVEL_OUT_OF_WINDOW_1_CB_ID = 0x02U, /*!< ADC analog watchdog 1 callback ID */
    HAL_ADC_ERROR_CB_ID = 0x03U, /*!< ADC error callback ID */
    HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID =
        0x04U, /*!< ADC group injected conversion complete callback ID */
    HAL_ADC_INJ_QUEUE_OVEFLOW_CB_ID =
        0x05U, /*!< ADC group injected context queue overflow callback ID */
    HAL_ADC_LEVEL_OUT_OF_WINDOW_2_CB_ID = 0x06U, /*!< ADC analog watchdog 2 callback ID */
    HAL_ADC_LEVEL_OUT_OF_WINDOW_3_CB_ID = 0x07U, /*!< ADC analog watchdog 3 callback ID */
    HAL_ADC_END_OF_SAMPLING_CB_ID = 0x08U, /*!< ADC end of sampling callback ID */
    HAL_ADC_MSPINIT_CB_ID = 0x09U, /*!< ADC Msp Init callback ID          */
    HAL_ADC_MSPDEINIT_CB_ID = 0x0AU /*!< ADC Msp DeInit callback ID        */
} HAL_ADC_CallbackIDTypeDef;

/**
 * @brief  HAL ADC Callback pointer definition
 */
typedef void (*pADC_CallbackTypeDef)(ADC_HandleTypeDef *hadc
); /*!< pointer to a ADC callback function */

// Mock function declarations - exact match with HAL signatures
// HAL_StatusTypeDef HAL_ADC_RegisterCallback(ADC_HandleTypeDef *hadc, HAL_ADC_CallbackIDTypeDef
// CallbackID, pADC_CallbackTypeDef pCallback)
DECLARE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_ADC_RegisterCallback, ADC_HandleTypeDef *, HAL_ADC_CallbackIDTypeDef,
    pADC_CallbackTypeDef
);

// HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t CalibrationMode,
// uint32_t SingleDiff)
DECLARE_FAKE_VALUE_FUNC(
    HAL_StatusTypeDef, HAL_ADCEx_Calibration_Start, ADC_HandleTypeDef *, uint32_t, uint32_t
);

// HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef *hadc)
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_ADC_Start_IT, ADC_HandleTypeDef *);

// HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef *hadc)
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_ADC_Stop_IT, ADC_HandleTypeDef *);

// uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc)
DECLARE_FAKE_VALUE_FUNC(uint32_t, HAL_ADC_GetValue, ADC_HandleTypeDef *);

#endif // MOCK_I2C_HAL_H
