#include "stm32h7xx_hal.h"

DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Read_IT,
                       I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                       uint8_t *, uint16_t);

DEFINE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Write_IT,
                       I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                       const uint8_t *, uint16_t);

DEFINE_FAKE_VOID_FUNC(HAL_I2C_RegisterCallback,
                      I2C_HandleTypeDef *, uint32_t, void *);

DEFINE_FAKE_VOID_FUNC(HAL_Init);

/* Remove these duplicate callback definitions if your driver provides them */
// DEFINE_FAKE_VOID_FUNC(HAL_I2C_MasterTxCpltCallback, I2C_HandleTypeDef*);
// DEFINE_FAKE_VOID_FUNC(HAL_I2C_MasterRxCpltCallback, I2C_HandleTypeDef*);
// DEFINE_FAKE_VOID_FUNC(HAL_I2C_ErrorCallback, I2C_HandleTypeDef*);
