#ifndef I2C_H
#define I2C_H

#include "rocketlib/include/common.h" // Common definitions
#include <stdint.h> // Standard integer types

// Supported I2C devices
typedef enum {
    I2C_DEVICE_POLULU, // Polulu AltIMU-v6
    I2C_DEVICE_ST, // ST LSM6DSO32 IMU
    I2C_DEVICE_COUNT // Number of devices
} i2c_device_t;

// Initialize an I2C device
// device: I2C device to initialize
// timeout_ms: Operation timeout in milliseconds
// Returns: Status code indicating success or failure
w_status_t i2c_init(i2c_device_t device, uint32_t timeout_ms);

// Read data from a device register
// device: I2C device
// reg: Register address
// data: Buffer to store read data
// len: Number of bytes to read (max 255)
// Returns: Status code indicating success or failure
w_status_t i2c_read_reg(i2c_device_t device, uint8_t reg, uint8_t *data, uint8_t len);

// Write data to a device register
// device: I2C device
// reg: Register address
// data: Data to write
// len: Number of bytes to write
// Returns: Status code indicating success or failure
w_status_t i2c_write_reg(i2c_device_t device, uint8_t reg, const uint8_t *data, uint8_t len);

#endif // I2C_H