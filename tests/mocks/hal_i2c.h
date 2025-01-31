#ifndef MOCKS_HAL_I2C_H
#define MOCKS_HAL_I2C_H

#include "fff.h"
#include "stm32h7xx_hal.h"

// FFF function declarations
FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Read_IT, I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);
FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Write_IT, I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);
FAKE_VOID_FUNC(HAL_I2C_RegisterCallback, I2C_HandleTypeDef *, HAL_I2C_CallbackIDTypeDef, void *);

void hal_i2c_mocks_init(void);
void hal_i2c_mocks_reset(void);

#endif // MOCKS_HAL_I2C_H