// tests/unit/i2c_test.cpp

extern "C"
{
#include "i2c.h"               // Your I2C driver header (from src/drivers/i2c)
#include "stm32h7xx_hal_i2c.h" // Our mock wrapper header (includes HAL mocks & dummy I2C handles)
#include "mock_freertos.h"     // Our FreeRTOS mocks (using our minimal stubs)
}

#include <gtest/gtest.h>

class I2CTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Reset all fake functions to a known state.
        RESET_FAKE(HAL_I2C_Mem_Read_IT);
        RESET_FAKE(HAL_I2C_Mem_Write_IT);
        RESET_FAKE(HAL_I2C_RegisterCallback);
        RESET_FAKE(HAL_Init);
        RESET_FAKE(xSemaphoreTake);
        RESET_FAKE(xSemaphoreGive);
        RESET_FAKE(xSemaphoreCreateMutex);
        RESET_FAKE(xSemaphoreCreateBinary);
        RESET_FAKE(vSemaphoreDelete);
        FFF_RESET_HISTORY();
    }
};

/**
 * Test initialization succeeds.
 */
TEST_F(I2CTest, InitSuccess)
{
    // Provide fake return values for semaphore creation.
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)0x1234;
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)0x5678;

    w_status_t status = i2c_init(I2C_BUS_1, 100);
    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(xSemaphoreCreateMutex_fake.call_count, 1);
    EXPECT_EQ(xSemaphoreCreateBinary_fake.call_count, 1);
}

/**
 * Test initialization failure when mutex creation fails.
 */
TEST_F(I2CTest, InitFailureMutexCreation)
{
    // Simulate mutex creation failure.
    xSemaphoreCreateMutex_fake.return_val = NULL;

    w_status_t status = i2c_init(I2C_BUS_1, 100);
    EXPECT_EQ(status, W_FAILURE);
    EXPECT_EQ(xSemaphoreCreateMutex_fake.call_count, 1);
}

/**
 * Test i2c_read_reg() when taking the bus mutex times out.
 * Note: This test calls i2c_init first so that the bus is initialized.
 */
TEST_F(I2CTest, ReadFailureDueToTimeout)
{
    // Setup initialization.
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)0x1234;
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)0x5678;
    w_status_t init_status = i2c_init(I2C_BUS_1, 100);
    EXPECT_EQ(init_status, W_SUCCESS);

    // Now simulate that when trying to take the mutex in read_reg, xSemaphoreTake returns pdFALSE.
    xSemaphoreTake_fake.return_val = pdFALSE;

    uint8_t data[4];
    w_status_t status = i2c_read_reg(I2C_BUS_1, 0x50, 0x10, data, sizeof(data));

    // Expect that read_reg returns a timeout code.
    EXPECT_EQ(status, W_IO_TIMEOUT);
    EXPECT_EQ(xSemaphoreTake_fake.call_count, 1);
}

/**
 * Test a successful i2c_read_reg() path.
 *
 * Here, after initialization, we simulate:
 * - xSemaphoreTake returning pdTRUE,
 * - HAL_I2C_Mem_Read_IT returning HAL_OK,
 * - and the transfer completion waiting (via xSemaphoreTake on transfer_sem) also returning pdTRUE.
 *
 * Since the same fake xSemaphoreTake is used for both the bus mutex and
 * the transfer semaphore in wait_transfer_complete, we expect its call count to be 2.
 */
TEST_F(I2CTest, ReadSuccess)
{
    // Set up proper initialization.
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)0x1234;
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)0x5678;
    w_status_t init_status = i2c_init(I2C_BUS_1, 100);
    EXPECT_EQ(init_status, W_SUCCESS);

    // Simulate that taking the bus and transfer semaphore succeed.
    xSemaphoreTake_fake.return_val = pdTRUE; // for both calls
    // Simulate that HAL I2C Mem Read returns HAL_OK.
    HAL_I2C_Mem_Read_IT_fake.return_val = HAL_OK;

    uint8_t data[4] = {0};
    w_status_t status = i2c_read_reg(I2C_BUS_1, 0x50, 0x10, data, sizeof(data));

    // Expect that:
    // - The fake xSemaphoreTake is called twice: once for the mutex and once for the transfer semaphore.
    EXPECT_EQ(xSemaphoreTake_fake.call_count, 2);
    // - The fake HAL_I2C_Mem_Read_IT is called once.
    EXPECT_EQ(HAL_I2C_Mem_Read_IT_fake.call_count, 1);
    // - And the bus mutex is released, so xSemaphoreGive is called once.
    EXPECT_EQ(xSemaphoreGive_fake.call_count, 1);

    // Optionally, check that the returned status is W_SUCCESS.
    EXPECT_EQ(status, W_SUCCESS);
}
