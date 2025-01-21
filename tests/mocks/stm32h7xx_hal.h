#ifndef STM32H7XX_HAL_H
#define STM32H7XX_HAL_H

#include "fff.h"
#include <stdint.h>

/* Minimal HAL status type */
typedef enum
{
    HAL_OK = 0x00U,
    HAL_ERROR = 0x01U,
    HAL_BUSY = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

/* Minimal I2C handle structure */
typedef struct
{
    int dummy; /* Placeholder member */
} I2C_HandleTypeDef;

/* Define the I2C memory address size macro */
#define I2C_MEMADD_SIZE_8BIT 1

/* Callback identifiers (if used) */
#define HAL_I2C_MASTER_TX_COMPLETE_CB_ID 0x01U
#define HAL_I2C_MASTER_RX_COMPLETE_CB_ID 0x02U

/* Fake function declarations using FFF */
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Read_IT,
                        I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                        uint8_t *, uint16_t);

DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Write_IT,
                        I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                        const uint8_t *, uint16_t);

DECLARE_FAKE_VOID_FUNC(HAL_I2C_RegisterCallback,
                       I2C_HandleTypeDef *, uint32_t, void *);

DECLARE_FAKE_VOID_FUNC(HAL_Init);

/* Optional stubs for callbacks */
DECLARE_FAKE_VOID_FUNC(HAL_I2C_MasterTxCpltCallback, I2C_HandleTypeDef *);
DECLARE_FAKE_VOID_FUNC(HAL_I2C_MasterRxCpltCallback, I2C_HandleTypeDef *);
DECLARE_FAKE_VOID_FUNC(HAL_I2C_ErrorCallback, I2C_HandleTypeDef *);

#endif // STM32H7XX_HAL_H
