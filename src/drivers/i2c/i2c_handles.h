#ifndef I2C_HANDLES_H
#define I2C_HANDLES_H

#include "stm32h7xx_hal.h"

/* Declare the global I2C handle variables that the driver expects.
   In a real system these would be defined by CubeMX (or similar configuration code). */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

#endif /* I2C_HANDLES_H */
