/**
 * @file mock_hal.c
 * @brief Implementation of HAL mocks
 */
#include "mock_hal.h"

// Define HAL I2C handles
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

// Define HAL mock functions
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Read_IT, I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Write_IT, I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);
DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_RegisterCallback, I2C_HandleTypeDef *, HAL_I2C_CallbackIDTypeDef, void *);

void mock_hal_Reset(void)
{
    RESET_FAKE(HAL_I2C_Mem_Read_IT);
    RESET_FAKE(HAL_I2C_Mem_Write_IT);
    RESET_FAKE(HAL_I2C_RegisterCallback);

    // Set default return values
    HAL_I2C_Mem_Read_IT_fake.return_val = HAL_OK;
    HAL_I2C_Mem_Write_IT_fake.return_val = HAL_OK;
    HAL_I2C_RegisterCallback_fake.return_val = HAL_OK;
}