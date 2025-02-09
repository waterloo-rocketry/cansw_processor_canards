// This file is the "entrypoint" for mocking the STM32H7xx HAL library.
#ifndef MOCK_STM32H7XX_HAL_H
#define MOCK_STM32H7XX_HAL_H

#include "fff.h"
// Include all other hal mocks, same way the actual hal library works (all src code only includes stm32h7xx_hal.h)
#include "hal_gpio_mock.h"
#include "hal_fdcan_mock.h"

// Declare (but do not define) mock here. Actual definition is in stm32h7xx_hal.c to avoid multiple-definitions errors
DECLARE_FAKE_VOID_FUNC(HAL_Init);

#endif // MOCK_STM32H7XX_HAL_H
