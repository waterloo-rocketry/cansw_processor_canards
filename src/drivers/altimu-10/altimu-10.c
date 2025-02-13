#include "drivers/altimu-10/altimu-10.h"

// AltIMU device addresses and configuration
#define LSM6DSO_ADDR 0x6B // default (addr sel pin vdd)
#define LIS3MDL_ADDR 0x1E // default (addr sel pin vdd)
#define LPS22DF_ADDR 0x5D // default (addr sel pin vdd)

/**
 * @brief Helper to read a sensor register.
 * @param reg_addr Register address to read from.
 * @param data Pointer to store the read data.
 * @return Status of the operation.
 */

// w_status_t altimu_read_register_with_timeout(uint8_t reg_addr, void *data);

/**
 * @brief Helper to write data to a sensor register.
 * @param reg_addr Register address to write to.
 * @param data Pointer to the data to write.
 * @return Status of the operation.
 */
// w_status_t altimu_write_register(uint8_t reg_addr, const void *data);
