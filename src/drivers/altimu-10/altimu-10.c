#include "drivers/altimu-10/altimu-10.h"
#include "drivers/altimu-10/LIS3MDL_regmap.h"
#include "drivers/altimu-10/LPS_regmap.h"
#include "drivers/altimu-10/LSM6DSO_regmap.h"
#include "drivers/i2c/i2c.h"

// AltIMU device addresses and configuration
#define LSM6DSO_ADDR 0x6B // default (addr sel pin vdd) IMU
#define LIS3MDL_ADDR 0x1E // default (addr sel pin vdd) Mag
#define LPS22DF_ADDR 0x5D // default (addr sel pin vdd) Baro

// Helper function for writing config (passing value as literal)
static w_status_t write_1_byte(uint8_t addr, uint8_t reg, uint8_t data) {
    return i2c_write_reg(I2C_BUS_4, addr, reg, &data, 1);
}

/**
 * @brief Performs a basic sanity check using the WHO_AM_I register.
 * @return Status of the operation.
 * TODO: improve error code verbosity?
 */
w_status_t altimu_check_sanity(void) {
    w_status_t i2c_status = W_SUCCESS;
    w_status_t device_status = W_SUCCESS;

    const uint8_t expected_lis3mdl = 0x3D;
    const uint8_t expected_lps22df = 0xB4;
    const uint8_t expected_lsm6dso = 0x6C;

    uint8_t who_am_i;
    i2c_status |= i2c_read_reg(I2C_BUS_4, LIS3MDL_ADDR, LIS3_WHO_AM_I, &who_am_i, 1);
    if (expected_lis3mdl != who_am_i) {
        device_status |= W_FAILURE;
    }

    i2c_status |= i2c_read_reg(I2C_BUS_4, LPS22DF_ADDR, LPS_WHO_AM_I, &who_am_i, 1);
    if (expected_lps22df != who_am_i) {
        device_status |= W_FAILURE;
    }

    i2c_status |= i2c_read_reg(I2C_BUS_4, LSM6DSO_ADDR, WHO_AM_I_REG, &who_am_i, 1);
    if (expected_lsm6dso != who_am_i) {
        device_status |= W_FAILURE;
    }

    if ((i2c_status != W_SUCCESS) || (device_status != W_SUCCESS)) {
        return W_FAILURE;
    }
    return W_SUCCESS;
}

/**
 * @brief Initializes the Pololu AltIMU incl all register configs
 * @note Must be called after scheduler start
 * @return Status of the operation
 */
w_status_t altimu_init() {
    w_status_t i2c_status = W_SUCCESS;
    // LSM6DSO: https://www.st.com/resource/en/datasheet/lsm6dso.pdf

    // Accel ODR: max (6.66 kHz)
    // Accel Fs: max  (+-16g)
    // LPF2_XL_EN: enable 2nd lowpass stage
    // Note: need to set XL_FS_MODE = 0
    i2c_status |= write_1_byte(LSM6DSO_ADDR, CTRL1_XL, 0xA6);

    // Gyro ODR: max (6.66 kHz)
    // Gyro Fs: max (+-2000 dps)
    i2c_status |= write_1_byte(LSM6DSO_ADDR, CTRL2_G, 0xAC);

    // BDU Enable
    i2c_status |= write_1_byte(LSM6DSO_ADDR, CTRL3_C, 0x44);

    // Gyro LPF1 Bw: 171.1 Hz (at 6.66 kHz ODR)
    // TODO: is auxiliary SPI disabled in hardware?
    i2c_status |= write_1_byte(LSM6DSO_ADDR, CTRL6_C, 0x02);

    // XL_FS_MODE = 0
    // Accel Lowpass Bandwidth = ODR/100
    i2c_status |= write_1_byte(LSM6DSO_ADDR, CTRL8_XL, 0x80);

    // Set bit 1 to 1 when not using I3C, recommended by datasheet
    i2c_status |= write_1_byte(LSM6DSO_ADDR, CTRL9_XL, 0x02);

    // LIS3MDL: https://www.pololu.com/file/0J1089/LIS3MDL.pdf

    // Enable Fast ODR (>80 Hz)
    // Mag ODR: max (1000 Hz)
    i2c_status |= write_1_byte(LIS3MDL_ADDR, LIS3_CTRL_REG1, 0x02);

    // Mag Fs: max (+- 16 gauss)
    i2c_status |= write_1_byte(LIS3MDL_ADDR, LIS3_CTRL_REG2, 0x60);

    // Continuous conversion mode
    i2c_status |= write_1_byte(LIS3MDL_ADDR, LIS3_CTRL_REG3, 0x0);

    // BDU enable
    i2c_status |= write_1_byte(LIS3MDL_ADDR, LIS3_CTRL_REG5, 0x40);

    // LPS22DF: https://www.pololu.com/file/0J1961/lps22df.pdf

    // ODR: max (200 Hz)
    //  Averaging: 8 samples
    i2c_status |= write_1_byte(LPS22DF_ADDR, LPS22DF_CTRL_REG1, 0x41);

    // LPF EN
    // LPF cutoff = ODR/4
    // BDU enable
    i2c_status |= write_1_byte(LPS22DF_ADDR, LPS22DF_CTRL_REG2, 0x18);

    return i2c_status == W_SUCCESS ? W_SUCCESS : W_FAILURE;
}

/**
 * @brief Retrieves accelerometer data.
 * @return Accelerometer data (gravities)
 */
w_status_t altimu_get_acc_data(vector3d_t *data);

/**
 * @brief Retrieves gyroscope data.
 * @return Gyroscope data (deg/s)
 */
w_status_t altimu_get_gyro_data(vector3d_t *data);

/**
 * @brief Retrieves magnetometer data.
 * @return Magnetometer data (gauss)
 */
w_status_t altimu_get_mag_data(vector3d_t *data);

/**
 * @brief Retrieves barometer data.
 * @return Barometer data (pascal, celsius)
 */
w_status_t altimu_get_baro_data(altimu_barometer_data_t *data);
