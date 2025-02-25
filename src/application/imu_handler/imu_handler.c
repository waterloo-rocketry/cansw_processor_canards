#include "application/imu_handler/imu_handler.h"
#include "application/estimator/estimator.h"
#include "drivers/altimu-10/altimu-10.h"
#include "drivers/lsm6dsv32x/lsm6dsv32x.h"
#include "drivers/movella/movella.h"
#include "drivers/timer/timer.h"

#include "FreeRTOS.h"
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
static struct
{
    bool initialized;
    uint32_t sample_count;
    uint32_t error_count;

    // Per-IMU stats
    struct
    {
        uint32_t success_count;
        uint32_t failure_count;
    } polulu_stats, st_stats, movella_stats;
} state = {0};

/**
 * @brief Initialize all IMUs
 * @return Status of initialization
 */
static w_status_t init_all_imus(void)
{
    w_status_t status = W_SUCCESS;

    status |= altimu_init();
    status |= lsm6dsv32_init();
    status |= movella_init();

    state.initialized = (status == W_SUCCESS);

    return status;
}
