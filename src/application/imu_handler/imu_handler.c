#include "application/imu_handler/imu_handler.h"
#include "FreeRTOS.h"
#include "application/estimator/estimator.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/movella/movella.h"
#include "drivers/timer/timer.h"
#include "task.h"

#include <string.h>

// Period of IMU sampling in milliseconds
#define IMU_SAMPLING_PERIOD_MS 5

// Timeout values for freshness check (in milliseconds)
#define GYRO_FRESHNESS_TIMEOUT_MS 5
#define MAG_FRESHNESS_TIMEOUT_MS 10
#define ACCEL_FRESHNESS_TIMEOUT_MS 5
#define BARO_FRESHNESS_TIMEOUT_MS 25

// Module state tracking
typedef struct {
    bool initialized;
    uint32_t sample_count;
    uint32_t error_count;

    // Per-IMU stats
    struct {
        uint32_t success_count;
        uint32_t failure_count;
    } polulu_stats, movella_stats;
} imu_handler_state_t;

static imu_handler_state_t imu_handler_state = {0};

/**
 * @brief Initialize all IMU hardware
 * @note Must be called after scheduler start
 * @return Status of the initialization operation (success only if all IMUs initialize)
 */
static w_status_t initialize_all_imus(void) {
    w_status_t status = W_SUCCESS;

    // First check if Polulu IMU is present
    if (W_SUCCESS != altimu_check_sanity()) {
        status = W_FAILURE;
    } else if (W_SUCCESS != altimu_init()) {
        status = W_FAILURE;
    }

    // Initialize Movella (no sanity check available yet)
    if (W_SUCCESS != movella_init()) {
        status = W_FAILURE;
    }

    imu_handler_state.initialized = (W_SUCCESS == status);
    return status;
}

/**
 * @brief Read data from the Polulu AltIMU-10 sensor
 * @param imu_data Pointer to store the IMU data
 * @return Status of the read operation
 */
static w_status_t read_pololu_imu(estimator_imu_measurement_t *imu_data) {
    w_status_t status = W_SUCCESS;

    // Read accelerometer, gyro, and magnetometer data
    status |= altimu_get_acc_data(&imu_data->accelerometer);
    status |= altimu_get_gyro_data(&imu_data->gyroscope);
    status |= altimu_get_mag_data(&imu_data->magnometer);

    // Read barometer data
    altimu_barometer_data_t baro_data;
    status |= altimu_get_baro_data(&baro_data);

    if (W_SUCCESS == status) {
        imu_data->barometer = baro_data.pressure;
        imu_data->is_dead = false;
        imu_handler_state.polulu_stats.success_count++;
    } else {
        // Set is_dead flag to indicate IMU failure
        imu_data->is_dead = true;
        imu_handler_state.polulu_stats.failure_count++;
    }

    return status;
}

/**
 * @brief Read data from the Movella MTi-630 sensor
 * @param imu_data Pointer to store the IMU data
 * @return Status of the read operation
 */
static w_status_t read_movella_imu(estimator_imu_measurement_t *imu_data) {
    w_status_t status;

    // Read all data from Movella in one call
    movella_data_t movella_data = {0}; // Initialize to zero
    status = movella_get_data(&movella_data, 100); // Add 100ms timeout

    if (W_SUCCESS == status) {
        // Copy data from Movella
        imu_data->accelerometer = movella_data.acc;
        imu_data->gyroscope = movella_data.gyr;
        imu_data->magnometer = movella_data.mag;
        imu_data->barometer = movella_data.pres;
        imu_data->is_dead = false;
        imu_handler_state.movella_stats.success_count++;
    } else {
        // Set is_dead flag to indicate IMU failure
        imu_data->is_dead = true;
        imu_handler_state.movella_stats.failure_count++;
    }

    return status;
}

/**
 * @brief Initialize the IMU handler module
 * @note This function is called before the scheduler starts
 * @return Status of initialization
 */
w_status_t imu_handler_init(void) {
    // Initialize module state
    memset(&imu_handler_state, 0, sizeof(imu_handler_state));

    // This function only initializes the module state
    // The actual IMU hardware initialization happens in the task
    return W_SUCCESS;
}

/**
 * @brief Execute one iteration of the IMU handler processing
 * Reads data from all IMUs and updates the estimator
 * @return Status of the execution
 */
w_status_t imu_handler_run(void) {
    estimator_all_imus_input_t imu_data = {
        .polulu = {.is_dead = false}, .movella = {.is_dead = false}
    };
    float current_time_ms;
    w_status_t status = W_SUCCESS;

    // Get current timestamp
    if (W_SUCCESS != timer_get_ms(&current_time_ms)) {
        current_time_ms = 0.0f;
    }
    uint32_t now_ms = (uint32_t)current_time_ms;

    // Set timestamps for all IMUs
    // Note: All IMUs get the same timestamp intentionally for synchronization
    imu_data.polulu.timestamp_imu = now_ms;
    imu_data.movella.timestamp_imu = now_ms;

    // Read from all IMUs and track their status
    w_status_t polulu_status = read_pololu_imu(&imu_data.polulu);
    w_status_t movella_status = read_movella_imu(&imu_data.movella);

    // If both IMUs fail, consider it a system-level failure
    if (W_FAILURE == polulu_status && W_FAILURE == movella_status) {
        status = W_FAILURE;
    }

    // Send data to estimator with status flags
    w_status_t estimator_status = estimator_update_imu_data(&imu_data);
    if (W_SUCCESS != estimator_status) {
        status = estimator_status;
        imu_handler_state.error_count++;
    }

    imu_handler_state.sample_count++;

    // Return overall status
    return status;
}

/**
 * @brief IMU handler task function for FreeRTOS
 * @note This task will be created during system initialization
 * @param argument Task argument (unused)
 */
void imu_handler_task(void *argument) {
    (void)argument; // Unused parameter

    // Initialize all IMUs - must be done after scheduler start
    w_status_t init_status = initialize_all_imus();
    if (W_SUCCESS != init_status) {
        // TODO: Add proper error logging/reporting here
        // Cannot proceed with IMU task if initialization fails
        // Consider adding a system-wide error handler or reset mechanism
        while (1) {
            // Infinite loop to prevent task from continuing
            vTaskDelay(portMAX_DELAY);
        }
    }

    // Variables for precise timing control
    TickType_t last_wake_time;
    const TickType_t frequency = pdMS_TO_TICKS(IMU_SAMPLING_PERIOD_MS);

    // Initialize last_wake_time to current time
    last_wake_time = xTaskGetTickCount();

    // Main task loop
    while (1) {
        w_status_t run_status = imu_handler_run();
        if (W_SUCCESS != run_status) {
            // Log or handle run failures if needed
            imu_handler_state.error_count++;
        }

        // Wait for next sampling period with precise timing
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}