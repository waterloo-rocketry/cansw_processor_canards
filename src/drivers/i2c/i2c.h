#ifndef I2C_H
#define I2C_H

#include "rocketlib/include/common.h"
#include <stdint.h>

typedef enum {
    I2C_DEVICE_POLULU, // Polulu AltIMU-v6
    I2C_DEVICE_MOVELLA, // Movella mti-630 IMU
    I2C_DEVICE_ST, // ST LSM6DSO32 IMU
    I2C_DEVICE_COUNT // Number of devices
} i2c_device_t;

// Must be called before RTOS scheduler starts
w_status_t i2c_init(i2c_device_t device, uint32_t timeout_ms);

// Read len bytes from reg into data buffer, max 255 byte
w_status_t i2c_read_reg(i2c_device_t device, uint8_t reg, uint8_t *data, uint8_t len);

w_status_t i2c_write_reg(i2c_device_t device, uint8_t reg, const uint8_t *data, uint8_t len);

#endif // I2C_H