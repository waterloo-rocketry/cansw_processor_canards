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

#include "rocketlib/include/common.h" // Common definitions
#include <stdint.h>                   // Standard integer types
#include <stdbool.h>

/**
 * @brief Available I2C buses in the system
 */
typedef enum
{
    I2C_BUS_1,    /**< I2C1 peripheral */
    I2C_BUS_2,    /**< I2C2 peripheral */
    I2C_BUS_3,    /**< I2C3 peripheral */
    I2C_BUS_COUNT /**< Number of available I2C buses */
} i2c_bus_t;

/**
 * @brief Initialize an I2C bus
 *
 * Initializes the specified I2C bus and creates required RTOS synchronization primitives.
 * Must be called before the RTOS scheduler starts.
 *
 * @param[in] bus The I2C bus to initialize
 * @param[in] timeout_ms Maximum time to wait for operations in milliseconds (0 for default)
 * @return Status code indicating success or failure
 * @retval W_SUCCESS Initialization successful
 * @retval W_INVALID_PARAM Invalid bus specified
 * @retval W_FAILURE Resource creation failed
 *
 * @note Each bus needs to be initialized exactly once before use
 */
w_status_t i2c_init(i2c_bus_t bus, uint32_t timeout_ms);

/**
 * @brief Read data from a device register
 *
 * Performs an I2C read transaction to read one or more registers from a device.
 * The function handles bus arbitration and will retry failed transfers.
 *
 * @param[in] bus The I2C bus to use
 * @param[in] device_addr 7-bit I2C device address
 * @param[in] reg Register address to read from
 * @param[out] data Buffer to store read data
 * @param[in] len Number of bytes to read (max 255)
 * @return Status code indicating success or failure
 * @retval W_SUCCESS Read completed successfully
 * @retval W_INVALID_PARAM Invalid parameters
 * @retval W_IO_TIMEOUT Transfer timed out
 * @retval W_IO_ERROR Bus error occurred
 *
 * @note The device_addr parameter expects the 7-bit address. The R/W bit is handled internally.
 */
w_status_t i2c_read_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief Write data to a device register
 *
 * Performs an I2C write transaction to write one or more registers to a device.
 * The function handles bus arbitration and will retry failed transfers.
 *
 * @param[in] bus The I2C bus to use
 * @param[in] device_addr 7-bit I2C device address
 * @param[in] reg Register address to write to
 * @param[in] data Data to write
 * @param[in] len Number of bytes to write (max 255)
 * @return Status code indicating success or failure
 * @retval W_SUCCESS Write completed successfully
 * @retval W_INVALID_PARAM Invalid parameters
 * @retval W_IO_TIMEOUT Transfer timed out
 * @retval W_IO_ERROR Bus error occurred
 *
 * @note The device_addr parameter expects the 7-bit address. The R/W bit is handled internally.
 */
w_status_t i2c_write_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, const uint8_t *data, uint8_t len);

#endif // I2C_H