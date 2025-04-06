/**
 * Extended Kalman Filter
 */
#ifndef EKF_H
#define EKF_H

#include "common/math/math.h"
#include "application/estimator/estimator_type.h"
#include "arm_math.h"
#include <math.h>

static x_state_t state;

void ekf_algorithm(x_state_t *state);

// define these here so we dont have to put the interesting part of the algo under 200 lines of matrix math
void ekf_predict();
void ekf_correct();

#endif