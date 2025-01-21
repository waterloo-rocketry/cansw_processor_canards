/**
 * @file mock_hal.h
 * @brief Mock definitions for STM32 HAL
 */
#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#include "fff.h"
#include <stdint.h>

// HAL Status type
typedef enum
{
    HAL_OK = 0x00,
    HAL_ERROR = 0x01,
    HAL_BUSY = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

// I2C Handle Structure (simplified for mocking)
typedef struct
{
    uint32_t Instance;  // I2C registers base address
    uint32_t State;     // I2C communication state
    uint32_t ErrorCode; // I2C Error code
} I2C_HandleTypeDef;

// Callback ID typedef
typedef enum
{
    HAL_I2C_MASTER_TX_COMPLETE_CB_ID = 0x00, /*!< I2C Master Tx Transfer completed callback ID */
    HAL_I2C_MASTER_RX_COMPLETE_CB_ID = 0x01, /*!< I2C Master Rx Transfer completed callback ID */
} HAL_I2C_CallbackIDTypeDef;

// External HAL handle declarations
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

// Declare HAL I2C mock functions
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Read_IT, I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Write_IT, I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_RegisterCallback, I2C_HandleTypeDef *, HAL_I2C_CallbackIDTypeDef, void *);

// Function to reset all HAL mocks
void mock_hal_Reset(void);

#endif // MOCK_HAL_H