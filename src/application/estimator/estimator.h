#ifndef STATE_EST_H
#define STATE_EST_H

#include "application/controller/controller.h"
#include "application/estimator/estimator_types.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

// measurement data from 1 arbitrary imu
typedef struct {
    uint32_t timestamp_imu;
    vector3d_t accelerometer; // gravities
    vector3d_t gyroscope; // rad/sec
    vector3d_t magnetometer; // mgauss (pololu) or arbitrary units (movella)
    float barometer; // Pa
    bool is_dead;
} estimator_imu_measurement_t;

// measurements from all imus together
typedef struct {
    estimator_imu_measurement_t pololu;
    estimator_imu_measurement_t movella;
} estimator_all_imus_input_t;

/**
 * @brief Used to update the imu inputs for estimator
 *
 * @note Should be called every 5ms or faster for optimal estimator performance
 *
 * @param data Pointer to the struct containing latest measurements from all imus
 */
w_status_t estimator_update_imu_data(estimator_all_imus_input_t *data);

/**
 * @brief initialize estimator module. call before creating estimator task
 */
w_status_t estimator_init();

/**
 * @brief Sends the complete state estimation data over CAN.
 *
 * Iterates through each state ID, builds a CAN message for it using the
 * current state data, and transmits it.
 *
 * @param current_state Pointer to the current state estimation data (x_state_t).
 * @return W_SUCCESS if all messages were sent successfully, W_FAILURE otherwise.
 */
w_status_t estimator_log_state_to_can(const x_state_t *current_state);

void estimator_task(void *argument);

#endif

