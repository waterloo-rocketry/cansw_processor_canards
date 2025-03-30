#include "../mocks/fff/fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "FreeRTOS.h"
#include "rocketlib/include/common.h"
#include "drivers/adc/adc.h"
#include "canlib.h"
#include "message_types.h"

#define E_WATCHDOG_TIMEOUT 0x81 //hardcoded for right now

//all the functions that are being tested 
extern w_status_t health_check_exec();
extern w_status_t health_check_init();
extern w_status_t get_adc_current(uint32_t *adc_current_mA);
extern void watchdog_register_task(TaskHandle_t task_handle, uint32_t timeout_ticks);
extern void watchdog_kick(void);
extern w_status_t check_watchdog_tasks(void);
extern void vTaskDelayUntil(TickType_t* pxPreviousWakeTime, const TickType_t xTimeIncrement);

FAKE_VALUE_FUNC(w_status_t, adc_get_value, adc_channel_t, uint32_t*, uint32_t);
FAKE_VALUE_FUNC(w_status_t, timer_get_ms, float*);
FAKE_VALUE_FUNC(w_status_t, can_handler_transmit, can_msg_t*);
FAKE_VALUE_FUNC5(bool, build_general_board_status_msg, can_msg_prio_t, uint16_t, uint32_t, uint16_t, can_msg_t*);
FAKE_VALUE_FUNC(TaskHandle_t, xTaskGetCurrentTaskHandle);
FAKE_VALUE_FUNC(TickType_t, xTaskGetTickCount);
FAKE_VOID_FUNC(vTaskDelayUntil, TickType_t*, TickType_t);

// Mocked global variables
static float timer_ms_value_mock;
static uint32_t adc_value_mock;

}

DEFINE_FFF_GLOBALS;

// Mocked functions for helping with unit testing 
// check if having extra static functions is good or not for unit testing
static w_status_t timer_get_ms_mock(float* out_time) {
    *out_time = timer_ms_value_mock;
    return W_SUCCESS;
}

static w_status_t adc_get_value_mock(adc_channel_t channel, uint32_t* out_value, uint32_t timeout_ms) {
    *out_value = adc_value_mock;
    return W_SUCCESS;
}

class HealthChecksTest : public ::testing::Test {
    protected:
        void SetUp() override {
            RESET_FAKE(adc_get_value);
            RESET_FAKE(timer_get_ms);
            RESET_FAKE(can_handler_transmit);
            RESET_FAKE(build_general_board_status_msg);
            RESET_FAKE(xTaskGetCurrentTaskHandle);
            RESET_FAKE(xTaskGetTickCount);
            RESET_FAKE(vTaskDelayUntil);
    
            FFF_RESET_HISTORY();
    
            adc_get_value_fake.return_val = W_SUCCESS;
            timer_get_ms_fake.return_val = W_SUCCESS;
            can_handler_transmit_fake.return_val = W_SUCCESS;
            build_general_board_status_msg_fake.return_val = true;
        }
    
        void TearDown() override {}

        // Helper functions to set up the test environment
        void SetTimerMs(float time_ms) {
            timer_ms_value_mock = time_ms;
            timer_get_ms_fake.custom_fake = timer_get_ms_mock;
        }
        
        // Set the ADC value to be returned by the fake function
        void SetAdcValue(uint32_t adc_val) {
            adc_value_mock = adc_val;
            adc_get_value_fake.custom_fake = adc_get_value_mock;
        }

        // Constants for mocking tests
        static constexpr float ADC_VREF = 3.3f;
        static constexpr float INA180A3_GAIN = 100.0f;
        static constexpr float R_SENSE = 0.033f;
        static constexpr uint16_t MAX_CURRENT_mA = 400;
        static constexpr uint32_t ADC_MAX_COUNTS_MOCK = 65535; //need another local definition for this
    };

TEST_F(HealthChecksTest, GetAdcCurrentPassing) {
    // Arrange
    uint32_t adc_current_mA;
    uint32_t adc_value = 1000;  // Example ADC value
    SetAdcValue(adc_value);
    
    
    float voltage_mV = ((float)(adc_value) / ADC_MAX_COUNTS_MOCK) * ADC_VREF * 1000.0f;
    uint32_t expected_current_mA = ((float)(voltage_mV) / INA180A3_GAIN) / R_SENSE; //doing the math again here
    
    // Act
    w_status_t result = get_adc_current(&adc_current_mA);
    

    // Assert
    EXPECT_EQ(W_SUCCESS, result);
    EXPECT_EQ(expected_current_mA, adc_current_mA);
    EXPECT_EQ(adc_get_value_fake.call_count, 1);
    EXPECT_EQ(adc_get_value_fake.arg0_val, PROCESSOR_BOARD_VOLTAGE);
}

TEST_F(HealthChecksTest, GetAdcCurrentFailing) {
    // Arrange
    uint32_t adc_current_mA;
    adc_get_value_fake.return_val = W_IO_TIMEOUT;
    
    // Act
    w_status_t result = get_adc_current(&adc_current_mA);
    
    // Assert
    EXPECT_EQ(W_IO_TIMEOUT, result);
    EXPECT_EQ(adc_get_value_fake.call_count, 1);
}

TEST_F(HealthChecksTest, NominalHealthCheck) {
    // Arrange
    SetTimerMs(1000.0f);
    uint32_t nominal_current = 300;  //example current
    SetAdcValue(nominal_current * R_SENSE * INA180A3_GAIN / (ADC_VREF * 1000.0f) * ADC_MAX_COUNTS_MOCK);
    
    // Act
    w_status_t result = health_check_exec();
    
    // Assert
    EXPECT_EQ(W_SUCCESS, result);
    EXPECT_EQ(build_general_board_status_msg_fake.call_count, 1);
    EXPECT_EQ(build_general_board_status_msg_fake.arg0_val, PRIO_LOW);
    EXPECT_EQ(build_general_board_status_msg_fake.arg2_val, E_NOMINAL);
}

TEST_F(HealthChecksTest, OvercurrentHealthCheck) {
    // Arrange
    SetTimerMs(1000.0f);
    uint32_t over_current = 5000;  //example too much current
    SetAdcValue(over_current * R_SENSE * INA180A3_GAIN / (ADC_VREF * 1000.0f) * ADC_MAX_COUNTS_MOCK);
    
    // Act
    w_status_t result = health_check_exec();
    
    // Assert
    EXPECT_EQ(W_SUCCESS, result);
    EXPECT_EQ(build_general_board_status_msg_fake.call_count, 1);
    EXPECT_EQ(build_general_board_status_msg_fake.arg0_val, PRIO_HIGH);
    EXPECT_EQ(build_general_board_status_msg_fake.arg2_val, E_5V_OVER_CURRENT);
}

TEST_F(HealthChecksTest, FailureHealthCheck) {
    // Arrange
    SetTimerMs(1000.0f);
    uint32_t nominal_current = 300;
    SetAdcValue(nominal_current * R_SENSE * INA180A3_GAIN / (ADC_VREF * 1000.0f) * ADC_MAX_COUNTS_MOCK);
    build_general_board_status_msg_fake.return_val = false;
    
    // Act
    w_status_t result = health_check_exec();
    
    // Assert
    EXPECT_EQ(W_FAILURE, result);
    EXPECT_EQ(build_general_board_status_msg_fake.call_count, 1);
}

TEST_F(HealthChecksTest, WatchdogRegisterAndKick) {
    // Arrange
    TaskHandle_t fake_task = (TaskHandle_t)0x12345678; //bs address
    uint32_t timeout_ticks = pdMS_TO_TICKS(100);
    SetTimerMs(1000.0f);
    xTaskGetCurrentTaskHandle_fake.return_val = fake_task;
    
    // Act
    watchdog_register_task(fake_task, timeout_ticks);
    watchdog_kick();
    
    // Assert
    EXPECT_EQ(timer_get_ms_fake.call_count, 2);  // Once for register, once for kick 
    EXPECT_EQ(xTaskGetCurrentTaskHandle_fake.call_count, 1);
    


    // Advance time but still within timeout becuase timout was earlier set to 100ms
    SetTimerMs(1050.0f);
    w_status_t result = check_watchdog_tasks();
    
    // Assert
    EXPECT_EQ(W_SUCCESS, result);
    EXPECT_EQ(build_general_board_status_msg_fake.call_count, 0);
}

TEST_F(HealthChecksTest, WatchdogTimeout) {
    // Arrange
    TaskHandle_t fake_task = (TaskHandle_t)0x12345678;
    uint32_t timeout_ticks = 100; 
    SetTimerMs(1000.0f);
    xTaskGetCurrentTaskHandle_fake.return_val = fake_task;

    watchdog_register_task(fake_task, timeout_ticks); //skip kicking to get to the timeout logic
    
    // Advance time beyond timeout
    SetTimerMs(1200.0f);  // 200ms later, should trigger timeout
    
    // Act
    w_status_t result = check_watchdog_tasks();
    
    // Assert
    EXPECT_EQ(W_SUCCESS, result);  
    EXPECT_EQ(build_general_board_status_msg_fake.call_count, 1);
    EXPECT_EQ(build_general_board_status_msg_fake.arg0_val, PRIO_HIGH);
    EXPECT_EQ(build_general_board_status_msg_fake.arg2_val, E_WATCHDOG_TIMEOUT);
}

TEST_F(HealthChecksTest, WatchdogMaxTasksLimit) {
    // Arrange
    TaskHandle_t fake_tasks[15];  // More than MAX_WATCHDOG_TASKS = 10
    for (uint8_t i = 0; i < 15; i++) {
        fake_tasks[i] = (TaskHandle_t)(uintptr_t)(0x10000 + i); //filling up with bs addresses
    }
    SetTimerMs(1000.0f);
    
    // Act
    for (int i = 0; i < 15; i++) {
        watchdog_register_task(fake_tasks[i], 100); //never would take more than 10 anyway
    }
    SetTimerMs(1050.0f);
    
    w_status_t result = check_watchdog_tasks();
    
    // Assert
    EXPECT_EQ(W_SUCCESS, result);  //check watchdog still goes through
    EXPECT_EQ(build_general_board_status_msg_fake.call_count, 0);
}
