#ifndef ALTIMU_10_H
#define ALTIMU_10_H

#include "rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

// Sensor data structure definitions
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} altimu_accelerometer_data;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} altimu_gyroscope_data;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} altimu_magnetometer_data;

typedef struct {
  float pressure;
  float temperature;
} altimu_barometer_data;

// IMU device addresses and configuration
typedef struct {
  uint8_t acc_addr;
  uint8_t gyr_addr;
  uint8_t mag_addr;
  uint8_t bar_addr;
  uint8_t who_am_i_reg;
} altimu_imu_address_info;

// Define the communication protocols
typedef enum {
  ALTIMU_I2C_PROTOCOL,
  ALTIMU_SPI_PROTOCOL,
  ALTIMU_UART_PROTOCOL,
  ALTIMU_UNKNOWN_PROTOCOL
} altimu_protocol;

// Function prototypes

/**
 * @brief Initializes the Pololu AltIMU module.
 * @param hi2c Pointer to the I2C handle.
 * @return Status of the operation.
 */
w_status_t altimu_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Detects the communication protocol and IMU addresses.
 * @param imu_info Pointer to altimu_imu_address_info structure to store
 * detected addresses.
 * @return Detected protocol type.
 */
altimu_protocol
altimu_detect_protocol_and_addresses(altimu_imu_address_info *imu_info);

/**
 * @brief Retrieves accelerometer data.
 * @return Accelerometer data as altimu_accelerometer_data structure.
 */
altimu_accelerometer_data altimu_get_acc_data(void);

/**
 * @brief Retrieves gyroscope data.
 * @return Gyroscope data as altimu_gyroscope_data structure.
 */
altimu_gyroscope_data altimu_get_gyr_data(void);

/**
 * @brief Retrieves magnetometer data.
 * @return Magnetometer data as altimu_magnetometer_data structure.
 */
altimu_magnetometer_data altimu_get_mag_data(void);

/**
 * @brief Retrieves barometer data.
 * @return Barometer data as altimu_barometer_data structure.
 */
altimu_barometer_data altimu_get_bar_data(void);

/**
 * @brief Performs a basic sanity check using the WHO_AM_I register.
 * @return Status of the operation.
 */
w_status_t altimu_check_sanity(void);

/**
 * @brief Verifies that the accelerometer data is within the expected range.
 * @param data Accelerometer data to check.
 * @return Status of the operation.
 */
w_status_t altimu_check_acc_data(altimu_accelerometer_data data);

/**
 * @brief Verifies that the gyroscope data is within the expected range.
 * @param data Gyroscope data to check.
 * @return Status of the operation.
 */
w_status_t altimu_check_gyr_data(altimu_gyroscope_data data);

/**
 * @brief Verifies that the magnetometer data is within the expected range.
 * @param data Magnetometer data to check.
 * @return Status of the operation.
 */
w_status_t altimu_check_mag_data(altimu_magnetometer_data data);

/**
 * @brief Verifies that the barometer data is within the expected range.
 * @param data Barometer data to check.
 * @return Status of the operation.
 */
w_status_t altimu_check_bar_data(altimu_barometer_data data);

/**
 * @brief Resets the AltIMU module.
 * @return Status of the operation.
 */
w_status_t altimu_reset(void);

/**
 * @brief Reads a sensor register with a timeout.
 * @param reg_addr Register address to read from.
 * @param data Pointer to store the read data.
 * @param timeout_ms Timeout in milliseconds.
 * @return Status of the operation.
 */
w_status_t altimu_read_register_with_timeout(uint8_t reg_addr, void *data,
                                             uint32_t timeout_ms);

/**
 * @brief Writes data to a sensor register.
 * @param reg_addr Register address to write to.
 * @param data Pointer to the data to write.
 * @return Status of the operation.
 */
w_status_t altimu_write_register(uint8_t reg_addr, const void *data);

/**
 * @brief Delays execution for a specified number of milliseconds.
 * @param ms Number of milliseconds to delay.
 */
void altimu_delay_ms(uint32_t ms);

// Constants and macros definitions
#define ALTIMU_WHO_AM_I_REG 0x0F      // WHO_AM_I register address
#define ALTIMU_EXPECTED_WHO_AM_I 0x6C // Expected WHO_AM_I value

// Below are subject to change...
extern I2C_HandleTypeDef hi2c1;   // I2C handler
extern SPI_HandleTypeDef hspi1;   // SPI handler
extern UART_HandleTypeDef huart2; // UART handler

#endif // ALTIMU_10_H
