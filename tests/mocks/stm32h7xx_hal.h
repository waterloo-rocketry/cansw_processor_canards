// This file is the "entrypoint" for mocking the STM32H7xx HAL library.
#ifndef MOCK_STM32H7XX_HAL_H
#define MOCK_STM32H7XX_HAL_H

#include "fff.h"

// mock relevant hal typedefs first, since the rest of the mocks depend on these
typedef enum
{
  HAL_OK       = 0x00,
  HAL_ERROR    = 0x01,
  HAL_BUSY     = 0x02,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

// Include all other hal mocks, same way the actual hal library works (all src code only includes stm32h7xx_hal.h)
// do this after declaring the global hal defs above cuz these depend on the global defs
#include "hal_gpio_mock.h"
#include "hal_i2c_mock.h"

// Declare (but do not define) mock here. Actual definition is in stm32h7xx_hal.c to avoid multiple-definitions errors
DECLARE_FAKE_VOID_FUNC(HAL_Init);

#endif // MOCK_STM32H7XX_HAL_H
