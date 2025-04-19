#include "drivers/altimu-10/altimu-10.h"
#include "application/logger/log.h"
#include "drivers/altimu-10/LIS3MDL_regmap.h"
#include "drivers/altimu-10/LPS_regmap.h"
#include "drivers/altimu-10/LSM6DSO_regmap.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include <limits.h>
#include <stdio.h>

// AltIMU device addresses and configuration
#define LSM6DSO_ADDR 0x6B // addr sel pin HIGH IMU
#define LIS3MDL_ADDR 0x1E // addr sel pin HIGH Mag
#define LPS22DF_ADDR 0x5D // addr sel pin HIGH Baro

// AltIMU conversion factors - based on config settings below
// TODO: verify against parameters tracking sheet
#define ACC_FS (16.0f / INT16_MAX) // g / LSB
#define GYRO_FS (2000.0f / INT16_MAX) // dps / LSB
#define MAG_FS (16.0f / INT16_MAX) // gauss / LSB
#define BARO_FS (100.0f / 4096.0f) // fixed scale: 100 Pa / 4096 LSB
#define TEMP_FS (1.0f / 100.0f) // fixed scale: 1 deg C / 100 LSB

// Helper function for writing config (passing value as literal)
static w_status_t write_1_byte(uint8_t addr, uint8_t reg, uint8_t data) {
    w_status_t status = i2c_write_reg(I2C_BUS_4, addr, reg, &data, 1);
    if (status != W_SUCCESS) {
        log_text(
            1,
            "AltIMU",
            "ERROR: I2C write failed (Addr: 0x%02X, Reg: 0x%02X, Status: %d)",
            addr,
            reg,
            status
        );
    }
    return status;
}

/**
 * @brief Performs a basic sanity check using the WHO_AM_I register.
 * @return Status of the operation.
 * TODO: improve error code verbosity?
 */
w_status_t altimu_check_sanity(void) {
    w_status_t i2c_status = W_SUCCESS;
    w_status_t device_status = W_SUCCESS;
    char error_msg[80]; // Buffer for specific error messages
    bool lis3mdl_ok = true;
    bool lps22df_ok = true;
    bool lsm6dso_ok = true;

    const uint8_t expected_lis3mdl = 0x3D;
    const uint8_t expected_lps22df = 0xB4;
    const uint8_t expected_lsm6dso = 0x6C;

    uint8_t who_am_i;
    i2c_status |= i2c_read_reg(I2C_BUS_4, LIS3MDL_ADDR, LIS3_WHO_AM_I, &who_am_i, 1);
    if (expected_lis3mdl != who_am_i) {
        device_status |= W_FAILURE;
        lis3mdl_ok = false;
    }

    i2c_status |= i2c_read_reg(I2C_BUS_4, LPS22DF_ADDR, LPS_WHO_AM_I, &who_am_i, 1);
    if (expected_lps22df != who_am_i) {
        device_status |= W_FAILURE;
        lps22df_ok = false;
    }

    i2c_status |= i2c_read_reg(I2C_BUS_4, LSM6DSO_ADDR, WHO_AM_I_REG, &who_am_i, 1);
    if (expected_lsm6dso != who_am_i) {
        device_status |= W_FAILURE;
        lsm6dso_ok = false;
    }

    if (i2c_status != W_SUCCESS) {
        log_text(1, "AltIMU", "ERROR: I2C read failed during sanity check.");
        // Don't return yet, check device status too
    }
    if (device_status != W_SUCCESS) {
        snprintf(
            error_msg,
            sizeof(error_msg),
            "ERROR: WHO_AM_I mismatch - LIS3MDL:%s LPS22DF:%s LSM6DSO:%s",
            lis3mdl_ok ? "OK" : "FAIL",
            lps22df_ok ? "OK" : "FAIL",
            lsm6dso_ok ? "OK" : "FAIL"
        );
        log_text(1, "AltIMU", error_msg);
    }

    if ((i2c_status != W_SUCCESS) || (device_status != W_SUCCESS)) {
        return W_FAILURE;
    }
    log_text(10, "AltIMU", "Sanity check passed.");
    return W_SUCCESS;
}

/**
 * @brief Initializes the Pololu AltIMU incl all register configs
 * @note Must be called after scheduler start
 * @return Status of the operation
 */
w_status_t altimu_init() {
    w_status_t status = W_SUCCESS;

    // Drive addr sel pin HIGH to use each device's "default" i2c addr
    status |= gpio_write(GPIO_PIN_ALTIMU_SA0, GPIO_LEVEL_HIGH, 10);
    if (status != W_SUCCESS) {
        log_text(1, "AltIMU", "ERROR: Failed to set GPIO Addr Sel pin.");
        // Continue attempting init, but log the error
    }

    // LSM6DSO: https://www.st.com/resource/en/datasheet/lsm6dso.pdf

    // Accel ODR: 208 Hz
    // Accel Fs: max  (+-16g)
    // LPF2_XL_EN: disable 2nd lowpass stage
    // Note: need to set XL_FS_MODE = 0
    status |= write_1_byte(LSM6DSO_ADDR, CTRL1_XL, 0x54);

    // Gyro ODR: 208 Hz
    // Gyro Fs: max (+-2000 dps)
    status |= write_1_byte(LSM6DSO_ADDR, CTRL2_G, 0x5C);

    // BDU Enable
    status |= write_1_byte(LSM6DSO_ADDR, CTRL3_C, 0x44);

    // Gyro LPF1 Bw: 67 Hz
    status |= write_1_byte(LSM6DSO_ADDR, CTRL6_C, 0x00);

    // XL_FS_MODE = 0
    // Accel Lowpass Bandwidth = ODR/2 = 104 Hz
    status |= write_1_byte(LSM6DSO_ADDR, CTRL8_XL, 0x0);

    // Set bit 1 to 1 when not using I3C, recommended by datasheet
    status |= write_1_byte(LSM6DSO_ADDR, CTRL9_XL, 0x02);

    // LIS3MDL: https://www.pololu.com/file/0J1089/LIS3MDL.pdf

    // Enable Fast ODR (>80 Hz)
    // Mag ODR: 155 Hz
    status |= write_1_byte(LIS3MDL_ADDR, LIS3_CTRL_REG1, 0x62);

    // Mag Fs: max (+- 16 gauss)
    status |= write_1_byte(LIS3MDL_ADDR, LIS3_CTRL_REG2, 0x60);

    // Continuous conversion mode
    status |= write_1_byte(LIS3MDL_ADDR, LIS3_CTRL_REG3, 0x0);

    // BDU enable
    status |= write_1_byte(LIS3MDL_ADDR, LIS3_CTRL_REG5, 0x40);

    // LPS22DF: https://www.pololu.com/file/0J1961/lps22df.pdf

    // ODR: max (200 Hz)
    //  Averaging: 8 samples
    status |= write_1_byte(LPS22DF_ADDR, LPS22DF_CTRL_REG1, 0x41);

    // LPF EN
    // LPF cutoff = ODR/4 = 50 Hz
    // BDU enable
    status |= write_1_byte(LPS22DF_ADDR, LPS22DF_CTRL_REG2, 0x18);

    if (status != W_SUCCESS) {
        log_text(1, "AltIMU", "ERROR: One or more register writes failed during init.");
    } else {
        log_text(10, "AltIMU", "Initialization successful.");
    }

    return status;
}

/**
 * @brief Retrieves accelerometer data.
 * @return Accelerometer data (gravities)
 */
w_status_t altimu_get_acc_data(vector3d_t *data) {
    uint8_t raw_data[6];
    w_status_t status = i2c_read_reg(I2C_BUS_4, LSM6DSO_ADDR, OUTX_L_A, raw_data, 6);
    if (W_SUCCESS != status) {
        log_text(1, "AltIMU", "ERROR: Failed to read Accel data (status: %d)", status);
    }
    // Data processing only if read was successful
    if (W_SUCCESS == status) {
        data->x = (int16_t)(((uint16_t)raw_data[1] << 8) | raw_data[0]) * ACC_FS;
        data->y = (int16_t)(((uint16_t)raw_data[3] << 8) | raw_data[2]) * ACC_FS;
        data->z = (int16_t)(((uint16_t)raw_data[5] << 8) | raw_data[4]) * ACC_FS;
    }
    return status;
}

/**
 * @brief Retrieves gyroscope data.
 * @return Gyroscope data (deg/s)
 */
w_status_t altimu_get_gyro_data(vector3d_t *data) {
    uint8_t raw_data[6];
    w_status_t status = i2c_read_reg(I2C_BUS_4, LSM6DSO_ADDR, OUTX_L_G, raw_data, 6);
    if (W_SUCCESS != status) {
        log_text(1, "AltIMU", "ERROR: Failed to read Gyro data (status: %d)", status);
    }
    // Data processing only if read was successful
    if (W_SUCCESS == status) {
        data->x = (int16_t)(((uint16_t)raw_data[1] << 8) | raw_data[0]) * GYRO_FS;
        data->y = (int16_t)(((uint16_t)raw_data[3] << 8) | raw_data[2]) * GYRO_FS;
        data->z = (int16_t)(((uint16_t)raw_data[5] << 8) | raw_data[4]) * GYRO_FS;
    }
    return status;
}

/**
 * @brief Retrieves magnetometer data.
 * @return Magnetometer data (gauss)
 */
w_status_t altimu_get_mag_data(vector3d_t *data) {
    uint8_t raw_data[6];
    w_status_t status = i2c_read_reg(I2C_BUS_4, LIS3MDL_ADDR, LIS3_OUT_X_L, raw_data, 6);
    if (W_SUCCESS != status) {
        log_text(1, "AltIMU", "ERROR: Failed to read Mag data (status: %d)", status);
    }
    // Data processing only if read was successful
    if (W_SUCCESS == status) {
        data->x = (int16_t)(((uint16_t)raw_data[1] << 8) | raw_data[0]) * MAG_FS;
        data->y = (int16_t)(((uint16_t)raw_data[3] << 8) | raw_data[2]) * MAG_FS;
        data->z = (int16_t)(((uint16_t)raw_data[5] << 8) | raw_data[4]) * MAG_FS;
    }
    return status;
}

/**
 * @brief Retrieves barometer data.
 * @return Barometer data (pascal, celsius)
 */
w_status_t altimu_get_baro_data(altimu_barometer_data_t *data) {
    uint8_t raw_data[5];
    w_status_t status = i2c_read_reg(I2C_BUS_4, LPS22DF_ADDR, LPS_PRESS_OUT_XL, raw_data, 5);
    if (W_SUCCESS != status) {
        log_text(1, "AltIMU", "ERROR: Failed to read Baro data (status: %d)", status);
    }
    // Data processing only if read was successful
    if (W_SUCCESS == status) {
        data->pressure =
            (int32_t)(((uint32_t)raw_data[2] << 16) | ((uint16_t)raw_data[1] << 8) | raw_data[0]) *
            BARO_FS;
        data->temperature = (int16_t)(((uint16_t)raw_data[4] << 8) | raw_data[3]) * TEMP_FS;
    }
    return status;
}
