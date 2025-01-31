/**
 * @file i2c.h
 * @brief Generic I2C bus driver interface
 *
 * This module provides low-level I2C communication functions for reading and writing
 * device registers. It handles bus access synchronization and error recovery but is
 * completely device-agnostic. Device-specific details like addresses and register
 * maps are handled by higher-level device drivers.
 */

#ifndef I2C_H
#define I2C_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "rocketlib/include/common.h"

/**
 * @brief Available I2C bus instances
 */
typedef enum {
    I2C_BUS_1,
    I2C_BUS_2,
    I2C_BUS_3,
    I2C_BUS_COUNT
} i2c_bus_t;

/**
 * @brief Initialize an I2C bus instance
 *
 * @param[in] bus Bus to initialize
 * @param[in] hal_handle STM32 HAL handle for this instance
 * @param[in] timeout_ms Transfer timeout in ms (0 for default)
 * @retval W_SUCCESS Initialization successful
 * @retval W_INVALID_PARAM Invalid bus or hal_handle
 * @retval W_FAILURE Resource allocation or callback registration failed
 */
w_status_t i2c_init(i2c_bus_t bus, I2C_HandleTypeDef *hal_handle, uint32_t timeout_ms);

/**
 * @brief Read from device registers
 *
 * @param[in] bus Bus to use
 * @param[in] device_addr 7-bit device address
 * @param[in] reg Starting register address
 * @param[out] data Buffer for read data
 * @param[in] len Number of bytes to read (max 255)
 * @retval W_SUCCESS Read completed successfully
 * @retval W_INVALID_PARAM Invalid parameters
 * @retval W_FAILURE Bus not initialized
 * @retval W_IO_TIMEOUT Transfer timed out
 * @retval W_IO_ERROR Bus error occurred
 */
w_status_t
i2c_read_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief Write to device registers
 *
 * @param[in] bus Bus to use
 * @param[in] device_addr 7-bit device address
 * @param[in] reg Starting register address
 * @param[in] data Data to write
 * @param[in] len Number of bytes to write (max 255)
 * @retval W_SUCCESS Write completed successfully
 * @retval W_INVALID_PARAM Invalid parameters
 * @retval W_FAILURE Bus not initialized
 * @retval W_IO_TIMEOUT Transfer timed out
 * @retval W_IO_ERROR Bus error occurred
 */
w_status_t
i2c_write_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, const uint8_t *data, uint8_t len);

#endif // I2C_H