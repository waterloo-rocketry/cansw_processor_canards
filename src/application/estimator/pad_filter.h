#ifndef PAD_FILTER_H
#define PAD_FILTER_H

#include "application/estimator/estimator.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

// bias results from pad_filter
typedef struct {
    float bias_1[10];
    float bias_2[10];
} sensor_bias_t;

// initial x from pad_filter
typedef struct {
    float x_init[13];
} x_init_t;

// Computes inital state and covariance estimate for EKF, and bias values for the IMU
w_status_t pad_filter(estimator_all_imus_input_t *data);

#endif