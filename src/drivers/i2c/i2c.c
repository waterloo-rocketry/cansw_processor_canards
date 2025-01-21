/**
 * @file i2c.c
 * @brief Implementation of the I2C bus driver
 */

#include "i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_i2c.h"

/**
 * @brief Default timeout value in milliseconds
 */
#define I2C_DEFAULT_TIMEOUT_MS 50

/**
 * @brief Maximum number of transfer retries
 */
#define I2C_MAX_RETRIES 3

/**
 * @brief I2C bus handle structure
 */
typedef struct
{
    I2C_HandleTypeDef *hal_handle;       /**< HAL handle from CubeMX */
    SemaphoreHandle_t mutex;             /**< Bus access mutex */
    SemaphoreHandle_t transfer_sem;      /**< Transfer completion semaphore */
    uint32_t timeout_ms;                 /**< Transfer timeout */
    volatile bool transfer_complete;     /**< Transfer completion flag */
    volatile w_status_t transfer_status; /**< Last transfer status */
} i2c_bus_handle_t;

/** @brief Module state for each I2C bus */
static i2c_bus_handle_t i2c_buses[I2C_BUS_COUNT];

/** @brief Initialization tracking for each bus */
static bool initialized[I2C_BUS_COUNT] = {false};

/**
 * @brief Error statistics for each bus
 */
static struct
{
    uint32_t timeouts;   /**< Number of timeout errors */
    uint32_t nacks;      /**< Number of NACK errors */
    uint32_t bus_errors; /**< Number of other bus errors */
} error_stats[I2C_BUS_COUNT] = {0};

// Forward declarations for internal functions
static void i2c_transfer_complete_callback(I2C_HandleTypeDef *hal_handle, bool success);
static w_status_t wait_transfer_complete(i2c_bus_handle_t *handle);

w_status_t i2c_init(i2c_bus_t bus, uint32_t timeout_ms)
{
    if (bus >= I2C_BUS_COUNT)
    {
        // TODO: Add logging when logger is available
        return W_INVALID_PARAM;
    }
    if (initialized[bus])
    {
        // TODO: Add logging when logger is available
        return W_FAILURE;
    }
    i2c_bus_handle_t *handle = &i2c_buses[bus]; // Get handle for the specified bus instance from
                                                // the array of bus handles in the module

    // Map bus enum to HAL handle
    switch (bus)
    {
    case I2C_BUS_1:
        handle->hal_handle = &hi2c1;
        break;
    case I2C_BUS_2:
        handle->hal_handle = &hi2c2;
        break;
    case I2C_BUS_3:
        handle->hal_handle = &hi2c3;
        break;
    default:
        return W_INVALID_PARAM;
    }
    handle->timeout_ms = timeout_ms > 0 ? timeout_ms : I2C_DEFAULT_TIMEOUT_MS;
    handle->transfer_complete = false;
    handle->transfer_status = W_SUCCESS;

    // Create RTOS synchronization primitives for the bus
    handle->mutex = xSemaphoreCreateMutex();
    if (!handle->mutex)
    {
        // TODO: Add logging when logger is available
        return W_FAILURE;
    }
    handle->transfer_sem = xSemaphoreCreateBinary();
    if (!handle->transfer_sem)
    {
        vSemaphoreDelete(handle->mutex); // Clean up the mutex if the semaphore creation failed to avoid resource leaks
        // TODO: Add logging when logger is available
        return W_FAILURE;
    }
    // Register the transfer completion callback with the HAL driver for the bus instance to handle
    // transfer completion events
    HAL_I2C_RegisterCallback(
        handle->hal_handle, HAL_I2C_MASTER_TX_COMPLETE_CB_ID, (void *)i2c_transfer_complete_callback);
    HAL_I2C_RegisterCallback(
        handle->hal_handle, HAL_I2C_MASTER_RX_COMPLETE_CB_ID, (void *)i2c_transfer_complete_callback);

    initialized[bus] = true; // Mark the bus as initialized
    // TODO: Add logging when logger is available
    return W_SUCCESS;
}

w_status_t
i2c_read_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (bus >= I2C_BUS_COUNT || !data || !len)
    {
        return W_INVALID_PARAM;
    }
    if (!initialized[bus])
    {
        // TODO: Add logging when logger is available
        return W_FAILURE;
    }

    i2c_bus_handle_t *handle = &i2c_buses[bus];
    w_status_t status = W_SUCCESS;
    uint8_t retries = 0;

    // Take bus mutex
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(handle->timeout_ms)) != pdTRUE)
    {
        error_stats[bus].timeouts++;
        // TODO: Add logging when logger is available
        return W_IO_TIMEOUT;
    }
    do
    {
        // Start the transfer
        handle->transfer_complete = false;
        HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read_IT(
            handle->hal_handle,
            device_addr << 1,     // Device address (HAL adds R/W bit)
            reg,                  // Device register address
            I2C_MEMADD_SIZE_8BIT, // Register address size
            data,
            len);
        if (hal_status != HAL_OK)
        {
            status = W_IO_ERROR;
            error_stats[bus].bus_errors++;
            // TODO: Add logging when logger is available
            break;
        }
        // Wait for transfer completion
        status = wait_transfer_complete(handle);
        if (status == W_SUCCESS)
        {
            break;
        }
        retries++;
        // TODO : Add logging when logger is available
    } while (retries < I2C_MAX_RETRIES);

    // Release bus mutex
    xSemaphoreGive(handle->mutex);
    return status;
}

w_status_t
i2c_write_reg(i2c_bus_t bus, uint8_t device_addr, uint8_t reg, const uint8_t *data, uint8_t len)
{
    if (bus >= I2C_BUS_COUNT || !data || !len)
    {
        return W_INVALID_PARAM;
    }

    if (!initialized[bus])
    {
        // TODO: Add logging when logger is available
        return W_FAILURE;
    }

    i2c_bus_handle_t *handle = &i2c_buses[bus];
    w_status_t status = W_SUCCESS;
    uint8_t retries = 0;

    // Take bus mutex
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(handle->timeout_ms)) != pdTRUE)
    {
        error_stats[bus].timeouts++;
        // TODO: Add logging when logger is available
        return W_IO_TIMEOUT;
    }

    do
    {
        // Start transfer
        handle->transfer_complete = false;
        HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Write_IT(
            handle->hal_handle,
            device_addr << 1, // Device address (HAL adds R/W bit)
            reg,              // Register address
            I2C_MEMADD_SIZE_8BIT,
            (uint8_t *)data,
            len);

        if (hal_status != HAL_OK)
        {
            status = W_IO_ERROR;
            error_stats[bus].bus_errors++;
            // TODO: Add logging when logger is available
            break;
        }

        // Wait for transfer completion
        status = wait_transfer_complete(handle);
        if (status == W_SUCCESS)
        {
            break;
        }

        retries++;
        // TODO: Add logging when logger is available
    } while (retries < I2C_MAX_RETRIES);

    xSemaphoreGive(handle->mutex);
    return status;
}

/**
 * @brief Callback function for transfer completion
 *
 * Called from ISR context when a transfer completes or fails.
 *
 * @param[in] hal_handle HAL I2C handle that completed
 * @param[in] success Whether the transfer was successful
 */
static void i2c_transfer_complete_callback(I2C_HandleTypeDef *hal_handle, bool success)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    // Find the corresponding bus handle
    for (int i = 0; i < I2C_BUS_COUNT; i++)
    {
        if (i2c_buses[i].hal_handle == hal_handle)
        {
            i2c_buses[i].transfer_complete = true;
            i2c_buses[i].transfer_status = success ? W_SUCCESS : W_IO_ERROR;

            // Give semaphore from ISR
            xSemaphoreGiveFromISR(i2c_buses[i].transfer_sem, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
            break;
        }
    }
}

/**
 * @brief Wait for transfer completion
 *
 * Waits for the transfer completion semaphore with timeout.
 *
 * @param[in] handle Bus handle to wait on
 * @return Status of the completed transfer
 */
static w_status_t wait_transfer_complete(i2c_bus_handle_t *handle)
{
    if (xSemaphoreTake(handle->transfer_sem, pdMS_TO_TICKS(handle->timeout_ms)) != pdTRUE)
    {
        handle->transfer_complete = true;
        handle->transfer_status = W_IO_TIMEOUT;
        return W_IO_TIMEOUT;
    }
    return handle->transfer_status; // Return the status of the completed transfer
}

// STM32 HAL I2C Event Interrupt Handlers
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c_transfer_complete_callback(hi2c, true);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c_transfer_complete_callback(hi2c, true);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    i2c_transfer_complete_callback(hi2c, false);
}