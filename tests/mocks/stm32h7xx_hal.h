#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H

#include "fff.h"
#include <stdint.h>

/* Minimal HAL status enumeration */
typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

/* Minimal definition for I2C handle.
   (In real code, this would be a more complex structure.) */
typedef struct
{
    int dummy;
} I2C_HandleTypeDef;

/* Define I2C memory address size macro */
#define I2C_MEMADD_SIZE_8BIT 1

#define HAL_I2C_MASTER_TX_COMPLETE_CB_ID 0x01U
#define HAL_I2C_MASTER_RX_COMPLETE_CB_ID 0x02U

/* Define the callback types that the HAL expects.
   (The real HAL defines these in stm32h7xx_hal_i2c.h; here we mimic them.) */
typedef uint32_t HAL_I2C_CallbackIDTypeDef;
typedef void (*pI2C_CallbackTypeDef)(I2C_HandleTypeDef *);

/* Declare fake functions using FFF macros.
   These will generate both the fake function and the associated _fake structure. */
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Read_IT,
                        I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);

DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Write_IT,
                        I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, const uint8_t *, uint16_t);

DECLARE_FAKE_VOID_FUNC(HAL_I2C_RegisterCallback,
                       I2C_HandleTypeDef *, HAL_I2C_CallbackIDTypeDef, pI2C_CallbackTypeDef);

DECLARE_FAKE_VOID_FUNC(HAL_Init);

#endif // STM32H7XX_HAL_H
