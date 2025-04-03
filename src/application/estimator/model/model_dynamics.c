#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/model/model_airdata.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// TODO: parameters go here as static const, or as define

// dynamics update function, returns the new integrated state
estimator_state_t
model_dynamics_update(float dt, estimator_state_t *est_state, estimator_input_t *est_input) {
    estimator_state_t state_new;

    // Compute rotation matrix from attitude quaternion
    matrix3d_t S = quaternion_rotmatrix(&(est_state->attitude));

    // Aerodynamics
    // get current air information (density is needed for aerodynamics)
    estimator_airdata_t airdata = model_airdata(est_state->altitude);
    float p_dyn = airdata.density / 2.0f * pow(quaternion_norm(est_state->velocity), 2);

    float sin_alpha = 0.0f, sin_beta = 0.0f;
    // angle of attack/sideslip
    if (abs(est_state->velocity.array[0]) >= 0.5) {
        sin_alpha =
            (est_state->velocity.array[2] / est_state->velocity.array[0]) /
            sqrt(pow(est_state->velocity.array[2], 2) / pow(est_state->velocity.array[0], 2) + 1);
        sin_beta =
            (est_state->velocity.array[1] / est_state->velocity.array[0]) /
            sqrt(pow(est_state->velocity.array[1], 2) / pow(est_state->velocity.array[0], 2) + 1);
    } else {
        if (est_state->velocity.array[2] == 0) {
            sin_alpha = 0;
        } else {
            est_state->velocity.array[2] > 0 ? sin_alpha = 1 : sin_alpha = -1;
        }
        if (est_state->velocity.array[1] == 0) {
            sin_beta = 0;
        } else {
            est_state->velocity.array[1] > 0 ? sin_beta = 1 : sin_beta = -1;
        }
    }

    // TODO torques
    // vector3d_t torque_canards = est_state.CL * est_state.delta *
    vector3d_t torque = {.array = {0, 0, 0}};

    // update attitude quaternion
    quaternion_t q_new =
        state.attitude + dt * quaternion_derivative(&est_state.attitude, &est_state.rates);
    state_new.attitude = quaternion_normalize(&q_new);

    // TODO rate update
    // state_new.rates = state.rates

    // TODO velocity update
    //vector3d_t v_new = est_state->velocity + 

    // altitude update 
    vector3d_t v_earth = math_vector3d_rotate(&math_matrix3d_transp(&S), &est_state.velocity); 
    state_new.altitude = est_state->altitude + dt * v_earth.array[0];

    // TODO canard coeff derivative

    // actuator dynamics
    // TODO linear 1st order 

    
    return state_new;
}