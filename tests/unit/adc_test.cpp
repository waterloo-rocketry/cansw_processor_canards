#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "FreeRTOS.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/i2c/i2c.h"

// // i2c_write_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, const uint8_t *data, uint8_t
// len); FAKE_VALUE_FUNC(w_status_t, i2c_write_reg, i2c_bus_t, uint8_t, uint8_t, const uint8_t *,
// uint8_t)

// // i2c_read_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len);
// FAKE_VALUE_FUNC(w_status_t, i2c_read_reg, i2c_bus_t, uint8_t, uint8_t, uint8_t *, uint8_t)
}

class ADCTest : public ::testing::Test {
protected:
    void SetUp() override {
        // RESET_FAKE(i2c_write_reg);
        // RESET_FAKE(i2c_read_reg);
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

TEST_F(ADCTest, InitCallsI2CWriteNTimes) {
    // // Arrange
    // i2c_write_reg_fake.return_val = W_SUCCESS;

    // // Act
    // w_status_t status = altimu_init();
    // // Assert
    // EXPECT_EQ(status, W_SUCCESS);

    // // There are currently 12 config registers we care about writing to when we initialize
    // // Checking that we do 12 writes is probably enough to detect if smth changes, I'm not going
    // to
    // // copy the current config into a test
    // EXPECT_EQ(i2c_write_reg_fake.call_count, 12);
    // EXPECT_EQ(i2c_write_reg_fake.arg0_val, I2C_BUS_4);
    // EXPECT_EQ(i2c_write_reg_fake.arg4_val, 1); // write length
}

