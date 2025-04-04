#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/model/quaternion.h"
#include "application/estimator/model/model_airdata.h"
#include "common/math/math.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// TODO: parameters go here as static const, or as define

// dynamics update function, returns the new integrated state
estimator_state_t model_dynamics_update(float dt, estimator_state_t *state, estimator_input_t *input)
{
    estimator_state_t state_new;

    // Compute rotation matrix from attitude quaternion
    matrix3d_t S = quaternion_rotmatrix(&(state->attitude));

    // Aerodynamics
    // get current air information (density is needed for aerodynamics)
    estimator_airdata_t airdata = model_airdata(state->altitude);
    // TODO: get proper aerodynamics running in matlab
    vector3d_t torque =  {.array = {0, 0, 0}}; 

    // update attiutde quaternion
    // TODO: get quaternion_increment to be running stable in Matlab, then use it here instead
    //quaternion_t q_new = state.attitude + dt * quaternion_derivative(state.attitude, state.rates);
    //state_new.attitude = quaternion_normalize(q_new);
    
    //state_new.rates = state.rates // TODO

    return state_new;
}