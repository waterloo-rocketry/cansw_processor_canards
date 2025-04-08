#ifndef PAD_FILTER_H
#define PAD_FILTER_H

#include "application/estimator/estimator.h"
#include "application/estimator/estimator_types.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @param IMU_1 input
 * @param IMU_2 input
 * @param is_dead_1 input - true means dead
 * @param is_dead_2 input - true means dead
 * @param x_init output
 * @param bias output
 */
w_status_t pad_filter(
    y_imu_t *IMU_1, y_imu_t *IMU_2, bool is_dead_1, bool is_dead_2, x_state_t *x_init,
    y_imu_t *bias_1, y_imu_t *bias_2
);
#endif