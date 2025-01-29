#include "hal_gpio_mock.h"

// Define FFF mocks for HAL_GPIO functions
DEFINE_FAKE_VALUE_FUNC(GPIO_PinState, HAL_GPIO_ReadPin, GPIO_TypeDef*, uint16_t);
DEFINE_FAKE_VOID_FUNC(HAL_GPIO_WritePin, GPIO_TypeDef*, uint16_t, GPIO_PinState);
DEFINE_FAKE_VOID_FUNC(HAL_GPIO_TogglePin, GPIO_TypeDef*, uint16_t);
