#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include "rocketlib/include/common.h"

/**
 * @brief Initialize the IMU handler module
 * Must be called before scheduler starts
 * @return Status of initialization
 */
w_status_t imu_handler_init(void);

/**
 * @brief Execute one iteration of the IMU handler processing
 * Reads data from all IMUs and updates the estimator
 * @return Status of the execution
 */
w_status_t imu_handler_run(void);

/**
 * @brief IMU handler task function for FreeRTOS
 * Should be created during system startup
 * @param argument Task argument (unused)
 */
void imu_handler_task(void *argument);

#endif // IMU_HANDLER_H