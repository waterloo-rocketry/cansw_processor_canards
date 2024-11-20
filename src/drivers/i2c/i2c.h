#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include "rocketlib/include/common.h"

typedef enum {
    I2C_PERIPHERAL_POLULU, // Polulu AltIMU-v6
    I2C_PERIPHERAL_MOVELLA, // Movella mti-630 IMU
    I2C_PERIPHERAL_ST, // ST LSM6DSO32 IMU
    I2C_PERIPHERAL_COUNT // Number of peripherals
} i2c_peripheral_t;

// Must be called before RTOS scheduler starts
w_status_t i2c_init(i2c_peripheral_t periph, uint32_t timeout_ms);

// Read len bytes from reg into data buffer, max 255 byte
w_status_t i2c_read_reg(i2c_peripheral_t periph, uint8_t reg, uint8_t *data, uint16_t len);

w_status_t i2c_write_reg(i2c_peripheral_t periph, uint8_t reg, uint8_t *data, uint16_t len);

#endif // I2C_H