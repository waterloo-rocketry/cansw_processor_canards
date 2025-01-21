#ifndef MOCK_I2C_HANDLES_H
#define MOCK_I2C_HANDLES_H

#include "stm32h7xx_hal.h" // This header (see below) defines I2C_HandleTypeDef

#ifdef __cplusplus
extern "C"
{
#endif

    extern I2C_HandleTypeDef hi2c1;
    extern I2C_HandleTypeDef hi2c2;
    extern I2C_HandleTypeDef hi2c3;

#ifdef __cplusplus
}
#endif

#endif /* MOCK_I2C_HANDLES_H */
