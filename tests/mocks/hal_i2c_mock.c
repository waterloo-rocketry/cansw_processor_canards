#include "hal_i2c_mock.h"

DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Read_IT,
                       I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                       uint8_t *, uint16_t);

DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Write_IT,
                       I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                       const uint8_t *, uint16_t);

DEFINE_FAKE_VOID_FUNC(HAL_I2C_RegisterCallback,
                      I2C_HandleTypeDef *, uint32_t, void *);

DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Master_Abort_IT,
                       I2C_HandleTypeDef *, uint16_t);

void hal_i2c_mocks_init(void)
{
    RESET_FAKE(HAL_I2C_Mem_Read_IT);
    RESET_FAKE(HAL_I2C_Mem_Write_IT);
    RESET_FAKE(HAL_I2C_RegisterCallback);
    RESET_FAKE(HAL_I2C_Master_Abort_IT);
}

void hal_i2c_mocks_reset(void)
{
    FFF_RESET_HISTORY();
    hal_i2c_mocks_init();
}