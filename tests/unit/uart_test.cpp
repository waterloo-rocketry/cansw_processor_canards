#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "drivers/uart/uart.h"
#include "hal_uart_mock.h"
#include "queue.h"
#include "stm32h7xx_hal.h"
}

// Helper function for queue receive simulation
static BaseType_t QueueReceiveCustomFake(QueueHandle_t queue, void *buffer, TickType_t wait) {
    uart_msg_t *msg = (uart_msg_t *)buffer;
    msg->len = 5;
    memcpy(msg->data, "test", 5);
    return pdTRUE;
}

// Test fixture for UART tests
class UartTest : public ::testing::Test {
protected:
    UART_HandleTypeDef huart;
    const uint32_t timeout = 100;

    void SetUp() override {
        // Reset all fakes before each test
        RESET_FAKE(xQueueCreate);
        RESET_FAKE(xQueueReceive);
        RESET_FAKE(HAL_UARTEx_ReceiveToIdle_IT);
        memset(&huart, 0, sizeof(huart));
    }
};

// Test initialization with valid parameters
TEST_F(UartTest, InitSuccess) {
    // Setup expected behavior
    xQueueCreate_fake.return_val = (QueueHandle_t)1; // Return valid queue handle
    HAL_UARTEx_ReceiveToIdle_IT_fake.return_val = HAL_OK;

    // Test initialization
    w_status_t status = uart_init(UART_MOVELLA, &huart, timeout);

    // Verify results
    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(1, xQueueCreate_fake.call_count);
    EXPECT_EQ(1, HAL_UARTEx_ReceiveToIdle_IT_fake.call_count);
}

// Test initialization with invalid parameters
TEST_F(UartTest, InitInvalidParams) {
    // Test with invalid channel
    w_status_t status = uart_init((uart_channel_t)99, &huart, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);

    // Test with null UART handle
    status = uart_init(UART_MOVELLA, nullptr, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);
}

// Test initialization failure cases
TEST_F(UartTest, InitFailures) {
    // Test queue creation failure
    xQueueCreate_fake.return_val = nullptr;
    w_status_t status = uart_init(UART_MOVELLA, &huart, timeout);
    EXPECT_EQ(W_FAILURE, status);

    // Test HAL initialization failure
    xQueueCreate_fake.return_val = (QueueHandle_t)1;
    HAL_UARTEx_ReceiveToIdle_IT_fake.return_val = HAL_ERROR;
    status = uart_init(UART_MOVELLA, &huart, timeout);
    EXPECT_EQ(W_IO_ERROR, status);
}

// Test uart_read with valid message
TEST_F(UartTest, ReadSuccess) {
    uint8_t buffer[MAX_MSG_LEN];
    uint16_t length;
    uart_msg_t test_msg = {.len = 5, .busy = true};
    memcpy(test_msg.data, "test", 5);

    // Setup xQueueReceive to return a message
    xQueueReceive_fake.custom_fake = QueueReceiveCustomFake;

    w_status_t status = uart_read(UART_MOVELLA, buffer, &length, timeout);

    EXPECT_EQ(W_SUCCESS, status);
    EXPECT_EQ(5, length);
    EXPECT_EQ(1, xQueueReceive_fake.call_count);
}

// Test uart_read with timeout
TEST_F(UartTest, ReadTimeout) {
    uint8_t buffer[MAX_MSG_LEN];
    uint16_t length;

    // Setup xQueueReceive to simulate timeout
    xQueueReceive_fake.return_val = pdFALSE;

    w_status_t status = uart_read(UART_MOVELLA, buffer, &length, timeout);

    EXPECT_EQ(W_IO_TIMEOUT, status);
    EXPECT_EQ(0, length);
    EXPECT_EQ(1, xQueueReceive_fake.call_count);
}

// Test uart_read with invalid parameters
TEST_F(UartTest, ReadInvalidParams) {
    uint8_t buffer[MAX_MSG_LEN];
    uint16_t length;

    // Test invalid channel
    w_status_t status = uart_read((uart_channel_t)99, buffer, &length, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);

    // Test null buffer
    status = uart_read(UART_MOVELLA, nullptr, &length, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);

    // Test null length pointer
    status = uart_read(UART_MOVELLA, buffer, nullptr, timeout);
    EXPECT_EQ(W_INVALID_PARAM, status);
}