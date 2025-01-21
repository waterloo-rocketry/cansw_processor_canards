#ifndef STM32H7XX_HAL_I2C_H
#define STM32H7XX_HAL_I2C_H

/* This header wraps our HAL mock and also provides dummy I²C handle declarations.
It makes sure that when the driver includes "stm32h7xx_hal_i2c.h", it sees all necessary definitions.
*/
#include "stm32h7xx_hal.h"    // Our HAL mocks and macro definitions
#include "mock_i2c_handles.h" // Declarations for hi2c1, hi2c2, and hi2c3

#endif /* STM32H7XX_HAL_I2C_H */
