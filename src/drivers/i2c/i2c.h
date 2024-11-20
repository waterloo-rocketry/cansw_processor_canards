#ifndef I2C_H
#define I2C_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef enum {
  I2C_PERIPHERAL_IMU,  // Pololu IMU
  I2C_PERIPHERAL_BARO, // Barometer
  I2C_PERIPHERAL_MAG   // Magnetometer
} i2c_peripheral_t;

typedef enum {
  I2C_OK = 0,  // Operation completed successfully
  I2C_ERROR,   // General error occurred
  I2C_BUSY,    // Bus or device busy
  I2C_TIMEOUT, // Operation timed out
  I2C_NACK     // Device did not acknowledge
} i2c_status_t;

// Must be called before RTOS scheduler starts
i2c_status_t i2c_init(i2c_peripheral_t periph, uint32_t timeout_ms);

// Read len bytes from reg into data buffer, max 255 byte
i2c_status_t i2c_read_reg(i2c_peripheral_t periph, uint8_t reg, uint8_t *data,
                          uint16_t len);

// Write len bytes from data buffer to reg, max 255 bytes
i2c_status_t i2c_write_reg(i2c_peripheral_t periph, uint8_t reg, uint8_t *data,
                           uint16_t len);

#endif // I2C_H