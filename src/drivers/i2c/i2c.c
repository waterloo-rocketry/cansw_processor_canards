/**
 * @file i2c.c
 * @brief Implementation of the I2C bus driver
 */

#include "i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"

/** @brief Default timeout for I2C operations in milliseconds */
#define I2C_DEFAULT_TIMEOUT_MS 50

/** @brief Maximum number of transfer retries before failing */
#define I2C_MAX_RETRIES 3

typedef struct i2c_bus_handle
{
    I2C_HandleTypeDef *hal_handle;       /**< STM32 HAL I2C handle */
    SemaphoreHandle_t mutex;             /**< Bus access mutex */
    SemaphoreHandle_t transfer_sem;      /**< Transfer completion synchronization */
    uint32_t timeout_ms;                 /**< Operation timeout in milliseconds */
    volatile bool transfer_complete;     /**< Transfer completion flag */
    volatile w_status_t transfer_status; /**< Result of last transfer operation */
    bool initialized;                    /**< Initialization status flag */
} i2c_bus_handle_t;

// Error statistics
i2c_error_data i2c_error_stats[I2C_BUS_COUNT] = {0};

// Initialize bus handles with default values
static i2c_bus_handle_t i2c_buses[I2C_BUS_COUNT] = {
    [0 ... I2C_BUS_COUNT - 1] = {
        .hal_handle = NULL,
        .mutex = NULL,
        .transfer_sem = NULL,
        .timeout_ms = I2C_DEFAULT_TIMEOUT_MS,
        .transfer_complete = false,
        .transfer_status = W_SUCCESS,
        .initialized = false}};

void i2c_transfer_complete_callback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    // Search for matching bus handle
    for (int i = 0; i < I2C_BUS_COUNT; i++)
    {
        if (i2c_buses[i].hal_handle == hi2c)
        {
            // First check for HAL-reported errors and update status
            if (hi2c->ErrorCode != HAL_I2C_ERROR_NONE)
            {
                // Handle NACK (address not acknowledged)
                if (hi2c->ErrorCode & HAL_I2C_ERROR_AF)
                {
                    i2c_error_stats[i].nacks++;
                }
                // Handle other bus errors
                else
                {
                    i2c_error_stats[i].bus_errors++;
                }
                i2c_buses[i].transfer_status = W_IO_ERROR;
                hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
            }
            else
            {
                // No errors - transfer completed successfully
                i2c_buses[i].transfer_status = W_SUCCESS;
            }

            i2c_buses[i].transfer_complete = true;
            xSemaphoreGiveFromISR(i2c_buses[i].transfer_sem, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
            return;
        }
    }
}

static w_status_t wait_transfer_complete(i2c_bus_handle_t *handle, i2c_bus_t bus)
{
    // Wait for transfer semaphore with timeout
    if (xSemaphoreTake(handle->transfer_sem, pdMS_TO_TICKS(handle->timeout_ms)) != pdTRUE)
    {
        // Timeout occurred - abort transfer and clean up
        HAL_I2C_Master_Abort_IT(handle->hal_handle, 0xFFFF);
        handle->transfer_complete = true;
        i2c_error_stats[bus].timeouts++;
        handle->transfer_status = W_IO_TIMEOUT;
    }

    // Return final transfer status (set by callback)
    return handle->transfer_status;
}

w_status_t i2c_init(i2c_bus_t bus, I2C_HandleTypeDef *hal_handle, uint32_t timeout_ms)
{
    // Validate input parameters
    if (bus >= I2C_BUS_COUNT || !hal_handle)
    {
        return W_INVALID_PARAM;
    }

    // Check if already initialized
    if (i2c_buses[bus].initialized)
    {
        return W_FAILURE;
    }

    // Get handle reference for cleaner code
    i2c_bus_handle_t *handle = &i2c_buses[bus];

    // Initialize handle fields
    handle->hal_handle = hal_handle;
    handle->timeout_ms = timeout_ms ? timeout_ms : I2C_DEFAULT_TIMEOUT_MS;
    handle->transfer_complete = false;
    handle->transfer_status = W_SUCCESS;

    // Create synchronization primitives
    handle->mutex = xSemaphoreCreateMutex();
    handle->transfer_sem = xSemaphoreCreateBinary();

    // Check if semaphore creation succeeded
    if (!handle->mutex || !handle->transfer_sem)
    {
        // Clean up any created resources
        if (handle->mutex)
        {
            vSemaphoreDelete(handle->mutex);
        }
        if (handle->transfer_sem)
        {
            vSemaphoreDelete(handle->transfer_sem);
        }
        return W_FAILURE;
    }

    // Register HAL callbacks for transfer events
    HAL_I2C_RegisterCallback(
        hal_handle, HAL_I2C_MEM_TX_COMPLETE_CB_ID, i2c_transfer_complete_callback);
    HAL_I2C_RegisterCallback(
        hal_handle, HAL_I2C_MEM_RX_COMPLETE_CB_ID, i2c_transfer_complete_callback);
    HAL_I2C_RegisterCallback(hal_handle, HAL_I2C_ERROR_CB_ID, i2c_transfer_complete_callback);

    // Mark bus as initialized
    handle->initialized = true;
    return W_SUCCESS;
}

w_status_t
i2c_read_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    // Validate input parameters
    if (bus >= I2C_BUS_COUNT || !data || !len)
    {
        return W_INVALID_PARAM;
    }

    // Get bus handle and check initialization
    i2c_bus_handle_t *handle = &i2c_buses[bus];
    if (!handle->initialized)
    {
        return W_FAILURE;
    }

    // Convert 7-bit device address to 8-bit HAL format by shifting left and setting R/W bit to 0
    uint16_t hal_addr = (device_addr << 1) & 0xFE;

    // Acquire bus mutex with timeout
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(handle->timeout_ms)) != pdTRUE)
    {
        i2c_error_stats[bus].timeouts++;
        return W_IO_TIMEOUT;
    }

    // Clear transfer state and prepare for new transfer
    handle->transfer_complete = false;
    handle->transfer_status = W_SUCCESS;     // Initialize with success status
    xSemaphoreGive(handle->transfer_sem);    // Give semaphore to ensure it's in a known state
    xSemaphoreTake(handle->transfer_sem, 0); // Clear any pending signal

    // Start non-blocking read operation
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read_IT(
        handle->hal_handle, hal_addr, reg, I2C_MEMADD_SIZE_8BIT, data, len);

    // Handle HAL-level errors
    if (hal_status != HAL_OK)
    {
        i2c_error_stats[bus].bus_errors++;
        xSemaphoreGive(handle->mutex);
        return W_IO_ERROR;
    }

    // Wait for transfer completion and store status
    w_status_t status = wait_transfer_complete(handle, bus);

    // Release bus mutex
    xSemaphoreGive(handle->mutex);
    return handle->transfer_status; // Return the status set by callback
}

w_status_t
i2c_write_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, const uint8_t *data, uint8_t len)
{
    // Validate input parameters
    if (bus >= I2C_BUS_COUNT || !data || !len)
    {
        return W_INVALID_PARAM;
    }

    // Get bus handle and check initialization
    i2c_bus_handle_t *handle = &i2c_buses[bus];
    if (!handle->initialized)
    {
        return W_FAILURE;
    }

    // Convert 7-bit device address to 8-bit HAL format by shifting left and setting R/W bit to 0
    uint16_t hal_addr = (device_addr << 1) & 0xFE;

    // Acquire bus mutex with timeout
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(handle->timeout_ms)) != pdTRUE)
    {
        i2c_error_stats[bus].timeouts++;
        return W_IO_TIMEOUT;
    }

    // Clear transfer state and prepare for new transfer
    handle->transfer_complete = false;
    handle->transfer_status = W_SUCCESS;     // Initialize with success status
    xSemaphoreGive(handle->transfer_sem);    // Give semaphore to ensure it's in a known state
    xSemaphoreTake(handle->transfer_sem, 0); // Clear any pending signal

    // Start non-blocking write operation
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Write_IT(
        handle->hal_handle, hal_addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len);

    // Handle HAL-level errors
    if (hal_status != HAL_OK)
    {
        i2c_error_stats[bus].bus_errors++;
        xSemaphoreGive(handle->mutex);
        return W_IO_ERROR;
    }

    // Wait for transfer completion and store status
    w_status_t status = wait_transfer_complete(handle, bus);

    // Release bus mutex
    xSemaphoreGive(handle->mutex);
    return handle->transfer_status; // Return the status set by callback
}