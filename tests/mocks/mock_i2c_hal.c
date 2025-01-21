// File: tests/mocks/mock_i2c_hal.c
#include "mock_i2c_hal.h"

DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Read_IT,
                       I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                       uint8_t *, uint16_t);

DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Write_IT,
                       I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                       const uint8_t *, uint16_t);

DEFINE_FAKE_VOID_FUNC(HAL_I2C_RegisterCallback,
                      I2C_HandleTypeDef *, uint32_t, void *);
