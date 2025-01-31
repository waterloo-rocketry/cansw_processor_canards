#include "gtest/gtest.h"
extern "C"
{
#include "i2c.h"
#include "mock_freertos.h"
#include "hal_i2c.h"
}

DEFINE_FFF_GLOBALS;

FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Read_IT, I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);
FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Write_IT, I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t, uint8_t *, uint16_t);
FAKE_VOID_FUNC(HAL_I2C_RegisterCallback, I2C_HandleTypeDef *, HAL_I2C_CallbackIDTypeDef, void *);
FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateMutex);
FAKE_VALUE_FUNC(SemaphoreHandle_t, xSemaphoreCreateBinary);
FAKE_VOID_FUNC(vSemaphoreDelete, SemaphoreHandle_t);
FAKE_VALUE_FUNC(BaseType_t, xSemaphoreTake, SemaphoreHandle_t, TickType_t);
FAKE_VALUE_FUNC(BaseType_t, xSemaphoreGive, SemaphoreHandle_t);

class I2CTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        RESET_FAKE(HAL_I2C_Mem_Read_IT);
        RESET_FAKE(HAL_I2C_Mem_Write_IT);
        RESET_FAKE(HAL_I2C_RegisterCallback);
        RESET_FAKE(xSemaphoreCreateMutex);
        RESET_FAKE(xSemaphoreCreateBinary);
        RESET_FAKE(vSemaphoreDelete);
        RESET_FAKE(xSemaphoreTake);
        RESET_FAKE(xSemaphoreGive);
        FFF_RESET_HISTORY();

        // Reset error stats
        memset(&error_stats, 0, sizeof(error_stats));
    }

    void TearDown() override
    {
        // Clean up any initialized buses
        for (int i = 0; i < I2C_BUS_COUNT; i++)
        {
            if (i2c_buses[i].initialized)
            {
                vSemaphoreDelete(i2c_buses[i].mutex);
                vSemaphoreDelete(i2c_buses[i].transfer_sem);
            }
        }
    }
};

TEST_F(I2CTest, InitSuccess)
{
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)0x1234;
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)0x5678;

    w_status_t status = i2c_init(I2C_BUS_1, &hi2c1, 100);

    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(xSemaphoreCreateMutex_fake.call_count, 1);
    EXPECT_EQ(xSemaphoreCreateBinary_fake.call_count, 1);
    EXPECT_EQ(HAL_I2C_RegisterCallback_fake.call_count, 3);
    EXPECT_TRUE(i2c_buses[I2C_BUS_1].initialized);
}

TEST_F(I2CTest, InitFailureMutexCreation)
{
    xSemaphoreCreateMutex_fake.return_val = NULL;

    w_status_t status = i2c_init(I2C_BUS_1, &hi2c1, 100);

    EXPECT_EQ(status, W_FAILURE);
    EXPECT_EQ(xSemaphoreCreateMutex_fake.call_count, 1);
    EXPECT_EQ(xSemaphoreCreateBinary_fake.call_count, 0);
}

TEST_F(I2CTest, ReadFailureDueToMutexTimeout)
{
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)0x1234;
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)0x5678;
    ASSERT_EQ(i2c_init(I2C_BUS_1, &hi2c1, 100), W_SUCCESS);

    xSemaphoreTake_fake.return_val = pdFALSE; // Mutex take fails

    uint8_t data[4];
    w_status_t status = i2c_read_reg(I2C_BUS_1, 0x50, 0x10, data, sizeof(data));

    EXPECT_EQ(status, W_IO_TIMEOUT);
    EXPECT_EQ(xSemaphoreTake_fake.call_count, 1);
    EXPECT_EQ(error_stats[I2C_BUS_1].timeouts, 1);
}

TEST_F(I2CTest, ReadSuccessWithCallback)
{
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)0x1234;
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)0x5678;
    ASSERT_EQ(i2c_init(I2C_BUS_1, &hi2c1, 100), W_SUCCESS);

    xSemaphoreTake_fake.return_val = pdTRUE; // For both mutex and transfer sem
    HAL_I2C_Mem_Read_IT_fake.return_val = HAL_OK;

    uint8_t data[4] = {0};
    w_status_t status = i2c_read_reg(I2C_BUS_1, 0x50, 0x10, data, sizeof(data));

    // Simulate transfer complete callback
    i2c_transfer_complete_callback(&hi2c1);

    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(HAL_I2C_Mem_Read_IT_fake.call_count, 1);
    EXPECT_EQ(xSemaphoreGive_fake.call_count, 1);
}

TEST_F(I2CTest, ReadRetriesOnHALError)
{
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)0x1234;
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)0x5678;
    ASSERT_EQ(i2c_init(I2C_BUS_1, &hi2c1, 100), W_SUCCESS);

    // First two attempts fail, third succeeds
    HAL_I2C_Mem_Read_IT_fake.return_val_seq = {HAL_ERROR, HAL_ERROR, HAL_OK};
    xSemaphoreTake_fake.return_val = pdTRUE;

    uint8_t data[4];
    w_status_t status = i2c_read_reg(I2C_BUS_1, 0x50, 0x10, data, sizeof(data));

    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(HAL_I2C_Mem_Read_IT_fake.call_count, 3);
    EXPECT_EQ(error_stats[I2C_BUS_1].bus_errors, 2);
}

TEST_F(I2CTest, WriteSuccess)
{
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)0x1234;
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)0x5678;
    ASSERT_EQ(i2c_init(I2C_BUS_1, &hi2c1, 100), W_SUCCESS);

    xSemaphoreTake_fake.return_val = pdTRUE;
    HAL_I2C_Mem_Write_IT_fake.return_val = HAL_OK;

    uint8_t data[4] = {0xAA};
    w_status_t status = i2c_write_reg(I2C_BUS_1, 0x50, 0x10, data, sizeof(data));

    // Simulate transfer complete callback
    i2c_transfer_complete_callback(&hi2c1);

    EXPECT_EQ(status, W_SUCCESS);
    EXPECT_EQ(HAL_I2C_Mem_Write_IT_fake.call_count, 1);
    EXPECT_EQ(xSemaphoreGive_fake.call_count, 1);
}

TEST_F(I2CTest, NackErrorHandling)
{
    xSemaphoreCreateMutex_fake.return_val = (SemaphoreHandle_t)0x1234;
    xSemaphoreCreateBinary_fake.return_val = (SemaphoreHandle_t)0x5678;
    ASSERT_EQ(i2c_init(I2C_BUS_1, &hi2c1, 100), W_SUCCESS);

    xSemaphoreTake_fake.return_val = pdTRUE;
    HAL_I2C_Mem_Read_IT_fake.return_val = HAL_OK;

    uint8_t data[4];
    i2c_read_reg(I2C_BUS_1, 0x50, 0x10, data, sizeof(data));

    // Simulate NACK error in callback
    hi2c1.ErrorCode = HAL_I2C_ERROR_AF;
    i2c_transfer_complete_callback(&hi2c1);

    EXPECT_EQ(error_stats[I2C_BUS_1].nacks, 1);
    EXPECT_EQ(error_stats[I2C_BUS_1].bus_errors, 0);
}

TEST_F(I2CTest, UninitializedBusAccess)
{
    uint8_t data[4];
    w_status_t status = i2c_read_reg(I2C_BUS_1, 0x50, 0x10, data, sizeof(data));
    EXPECT_EQ(status, W_FAILURE);

    status = i2c_write_reg(I2C_BUS_1, 0x50, 0x10, data, sizeof(data));
    EXPECT_EQ(status, W_FAILURE);
}