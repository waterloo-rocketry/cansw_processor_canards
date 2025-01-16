#ifndef I2C_H
#define I2C_H

#include "common.h" // Common definitions
#include <stdint.h> // Standard integer types

/**
 * @brief Supported I2C devices.
 */
typedef enum {
    I2C_DEVICE_POLULU, /**< Pololu AltIMU-v6 */
    I2C_DEVICE_ST, /**< ST LSM6DSO32 IMU */
    I2C_DEVICE_COUNT /**< Number of devices */
} i2c_device_t;

/**
 * @brief Initialize an I2C device.
 *
 * @param device I2C device to initialize.
 * @param timeout_ms Operation timeout in milliseconds.
 * @return Status code indicating success or failure.
 */
w_status_t i2c_init(i2c_device_t device, uint32_t timeout_ms);

/**
 * @brief Read data from a device register.
 *
 * @param device I2C device.
 * @param reg Register address.
 * @param data Pointer to data buffer.
 * @param len Number of bytes to read (max 255).
 * @return Status code indicating success or failure.
 */
w_status_t i2c_read_reg(i2c_device_t device, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief Write data to a device register.
 *
 * @param device I2C device.
 * @param reg Register address.
 * @param data Data to write.
 * @param len Number of bytes to write.
 * @return Status code indicating success or failure.
 */
w_status_t i2c_write_reg(i2c_device_t device, uint8_t reg, const uint8_t *data, uint8_t len);

#endif // I2C_H