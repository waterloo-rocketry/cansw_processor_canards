#include "hal_i2c.h"

void hal_i2c_mocks_init(void)
{
    RESET_FAKE(HAL_I2C_Mem_Read_IT);
    RESET_FAKE(HAL_I2C_Mem_Write_IT);
    RESET_FAKE(HAL_I2C_RegisterCallback);
}

void hal_i2c_mocks_reset(void)
{
    FFF_RESET_HISTORY();
    hal_i2c_mocks_init();
}