/**
 * Extended Kalman Filter
 */
#ifndef EKF_H
#define EKF_H

#include "common/math/math.h"
#include "arm_math.h"
#include <math.h>

typedef union {
    float array[13];
    struct {
        quaternion_t attitude;
        vector3d_t rates;
        vector3d_t velocity;
        float altitude;
        float CL;
        float delta;
    }
} est_state_t;

typedef struct {
    float cmd;
    vector3d_t acceleration;
} est_input_t;

static est_state_t state;

void ekf_algorithm(est_state_t *state);

void ekf_predict();

void ekf_correct();

#endif