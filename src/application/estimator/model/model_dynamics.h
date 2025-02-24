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


typedef struct {
    quaternion_t attitude; // Current attitude vector
    vector3d_t rates;      // Current angular rates
    vector3d_t velocity;   // Current velocity vector
    float altitude;        // Current altitude
    float canard_CL;       // Canard coefficient
    float canard_delta;    // Canard angle
} estimator_state_t;

typedef struct {
    vector3d_t acceleration; // Averaged specific force, from acceleration model
    float canard_command;    // Canard angle
} estimator_input_t;

estimator_state_t model_dynamics_update(float dt, estimator_state_t state, estimator_input_t input);

float[13][13] model_dynamics_jacobian(float dt, estimator_state_t state, estimator_input_t input);

float[13][13] model_dynamics_weights();

#endif