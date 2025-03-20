#ifndef STATE_EST_H
#define STATE_EST_H

#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

// measurement data from 1 arbitrary imu
typedef struct {
    uint32_t timestamp_imu;
    vector3d_t accelerometer;
    vector3d_t gyroscope;
    vector3d_t magnometer;
    float barometer;
    bool is_dead;
} estimator_imu_measurement_t;

// measurements from all imus together
typedef struct {
    estimator_imu_measurement_t polulu;
    estimator_imu_measurement_t movella;
} estimator_all_imus_input_t;

/**
 * @brief Used to update the imu inputs for estimator
 *
 * @note Should be called every 5ms or faster for optimal estimator performance
 *
 * @param data Pointer to the struct containing latest measurements from all imus
 */
w_status_t estimator_update_inputs_imu(estimator_all_imus_input_t *data);

void estimator_task(void *argument);

w_status_t estimator_init();

#endif