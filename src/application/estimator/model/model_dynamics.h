#ifndef MODEL_DYNAMICS_H
#define MODEL_DYNAMICS_H

#include "application/estimator/model/quaternion.h"
#include "application/estimator/model/model_airdata.h"
#include "application/estimator/estimator.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// definitions are here temporarily for easy lookup
typedef union {
    float array[13];
    struct {
        quaternion_t attitude;  // Attitude quaternion
        vector3d_t rates;       // Angular rates, body frame
        vector3d_t velocity;    // Velocity vector, body frame
        float altitude;         // Altitude
        float CL;               // Canard coefficient
        float delta;            // Canard angle
    }
} estimator_state_t;

typedef struct {
    vector3d_t acceleration; // Averaged specific force, from acceleration model
    float canard_command;    // Canard angle
} estimator_input_t;

estimator_state_t model_dynamics_update(float dt, estimator_state_t *est_state, estimator_input_t *est_input);

void model_dynamics_jacobian(float *dynamics_jacobian, float dt, estimator_state_t *state, estimator_input_t *input);

// void model_dynamics_weights(float *dynamics_weights);

#endif