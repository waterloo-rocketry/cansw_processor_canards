#ifndef MODEL_DYNAMICS_H
#define MODEL_DYNAMICS_H


#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_airdata.h"
#include "third_party/rocketlib/include/common.h"
#include "arm_math.h"


/**
 * Model dynamics update (non-jacobian)
 * Computes state derivative with predictive model
 * @param x_state_t current state of estimator
 * @param u_dynamics_t input signal
 * @param double THE dt time step
 * @return x_state_t new state of estimator
 */
x_state_t model_dynamics_update(const x_state_t *state, const u_dynamics_t *input, double dt);

/**
 * Model dynamics jacobian
 * Computes jacobian of state derivative with respect to state and input
 * @param arm_matrix_instance_f32  jacobian matrix to write to
 * @param x_state_t current state of estimator
 * @param u_dynamics_t input signal
 * @param double THE dt time step
 */
void model_dynamics_jacobian(
    const arm_matrix_instance_f32 *dynamics_jacobian, const x_state_t *state,
    const u_dynamics_t *input, double dt
);

// void model_dynamics_weights(float *dynamics_weights);

#endif