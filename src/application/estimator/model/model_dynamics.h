#ifndef MODEL_DYNAMICS_H
#define MODEL_DYNAMICS_H

#include "application/estimator/estimator_types.h"
#include "application/estimator/model/quaternion.h"
#include "application/estimator/model/model_airdata.h"
#include "application/estimator/estimator.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>


x_state_t model_dynamics_update(x_state_t *state, u_dynamics_t *input, uint32_t timestamp);

<<<<<<< HEAD

void model_dynamics_jacobian(arm_matrix_instance_f32 *dynamics_jacobian, x_state_t *state, u_dynamics_t *input, uint32_t timestamp);

=======
void model_dynamics_jacobian(arm_matrix_instance_f32 *dynamics_jacobian, x_state_t *state, u_dynamics_t *input, uint32_t timestamp);
>>>>>>> a35997935f271e96571e70b157ac9ae90e1d49f5

// void model_dynamics_weights(float *dynamics_weights);

#endif