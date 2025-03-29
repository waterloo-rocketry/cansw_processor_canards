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

est_state_t state;
est_state_t state_new;

void ekf_algorithm(est_state_t *state_new, const est_state_t *state);

void ekf_predict(est_state_t *state_new, const est_state_t *state);

void ekf_correct(est_state_t *state_new, const est_state_t *state);

#endif