// File: tests/mocks/mock_i2c_hal.h
#ifndef MOCK_I2C_HAL_H
#define MOCK_I2C_HAL_H

#include "fff.h"
#include "stm32h7xx_hal.h" // Ensure this header (or a stub) is available

// Fake function for non-blocking memory read (Interrupt-driven)
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Read_IT,
                        I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                        uint8_t *, uint16_t);

// Fake function for non-blocking memory write (Interrupt-driven)
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Write_IT,
                        I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                        const uint8_t *, uint16_t);

// Fake function for registering a callback.
// If your driver uses this, even if it’s a no-op in test, you still want to override it.
DECLARE_FAKE_VOID_FUNC(HAL_I2C_RegisterCallback,
                       I2C_HandleTypeDef *, uint32_t, void *);

#endif // MOCK_I2C_HAL_H
