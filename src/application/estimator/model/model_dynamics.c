#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_airdata.h"
#include "application/estimator/model/quaternion.h"
#include "application/logger/log.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "drivers/timer/timer.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/*
 * Parameters
 * go here as static const, or as define
 */
// aerodynamic parameters
static const float c_canard =
    (2 * (4 * 0.0254) * (2.5 * 0.0254)) *
    (0.203 / 2 + 0.0254); // moment arm * area of canard 
static const float cn_alpha = 5.0f; // pitch forcing coeff
static const float cn_omega = 0.0f; // roll damping coeff
static const float area_reference = M_PI * powf((0.203 / 2), 2); // cross section of body tube
// c_aero = area_reference * (length_cp-length_cg), center of pressure(cp): -0.5, center of
// gravity(cg): 0
static const float c_aero = area_reference * (-0.5f);

// mass and inertia
static const matrix3d_t inertia_matrix = {
    .array = {{0.17, 0, 0}, {0, 11.0, 0}, {0, 0, 11.0}}
}; // inertia matrix of the rocket (J in matlab)
static const matrix3d_t inertia_matrix_inv = {
    .array = {{1 / 0.17, 0, 0}, {0, 1 / 11.0, 0}, {0, 0, 1 / 11.0}}
}; // inverse of inertia matrix of the rocket (J^-1 in matlab)
// gravitational acceleration in the geographic inertial frame (g in matlab)
static vector3d_t grav_acc = {.array = {-9.81, 0, 0}};

// actuator and airfoil
static const float tau_cl_alpha = 2.0f; // time constant to converge Cl back to 1.5 in filter
static const float cl_alpha = 5.0f; // estimated coefficient of lift, const with Ma
static const float tau = 1 / 20.0f; // time constant of first order actuator dynamics

// initialize

// timestamp
#define MS_TO_SECONDS 1000.0
static float prev_timestamp_ms = 0.0, current_timestamp_ms = 0.0; // timestamp of function call
static double dt =
    0.0; // in seconds, using double for precision (only in model_dynamic_update and jacobians)

/*
 * Dynamics update
 * returns the new integrated state
 */
x_state_t model_dynamics_update(x_state_t *state, u_dynamics_t *input) {
    // update dt
    if (W_SUCCESS == timer_get_ms(&current_timestamp_ms)) {
        dt = (current_timestamp_ms - prev_timestamp_ms) / MS_TO_SECONDS;
        prev_timestamp_ms = current_timestamp_ms;
    }

    x_state_t state_new;

    // Compute rotation matrix from attitude quaternion

    matrix3d_t S = quaternion_rotmatrix(&(state->attitude));

    matrix3d_t ST = math_matrix3d_transp(&S); // S'

    /*
     * Aerodynamics
     */
    // get current air information (density is needed for aerodynamics)
    estimator_airdata_t airdata = model_airdata(state->altitude);
    float p_dyn = airdata.density / 2.0f * powf(math_vector3d_norm(&(state->velocity)), 2);

    float sin_alpha = 0.0f, sin_beta = 0.0f;
    // angle of attack/sideslip
    if (abs(state->velocity.x) >= 0.5) {
        sin_alpha = (state->velocity.z / state->velocity.x) /
                    sqrtf(powf(state->velocity.z, 2) / powf(state->velocity.x, 2) + 1);
        sin_beta = (state->velocity.y / state->velocity.x) /
                   sqrtf(powf(state->velocity.y, 2) / powf(state->velocity.x, 2) + 1);
    } else {
        if (state->velocity.z == 0) {
            sin_alpha = 0;
        } else {
            sin_alpha = (state->velocity.z > 0) ? 1 : -1;
        }
        if (state->velocity.y == 0) {
            sin_beta = 0;
        } else {
            sin_beta = (state->velocity.y > 0) ? 1 : -1;
        }
    }

    // torque calculations
    vector3d_t torque_unit_x = {.array = {1, 0, 0}};
    vector3d_t torque_sin_yz = {.array = {0, cn_alpha * sin_alpha, -cn_alpha * sin_beta}};
    // param.Cn_alpha*[0; sin_alpha; -sin_beta]
    vector3d_t torque_omega_yz = {
        .array = {0, cn_omega * state->rates.y, cn_omega * state->rates.z}
    }; // param.Cn_omega*[0; w(2); w(3)]
    vector3d_t torque_yz = math_vector3d_add(
        &torque_sin_yz,
        &torque_omega_yz
    ); // param.Cn_alpha*[0; sin_alpha; -sin_beta] + param.Cn_omega*[0; w(2); w(3)]
    vector3d_t torque_canards =
        math_vector3d_scale(state->CL * state->delta * c_canard * p_dyn, &torque_unit_x);
    vector3d_t torque_aero = math_vector3d_scale(p_dyn * c_aero, &torque_yz);
    vector3d_t torque = math_vector3d_add(&torque_canards, &torque_aero);

    // update attitude quaternion
    // dq = quaternion_derivative(q, w)
    quaternion_t dq = quaternion_derivative(&state->attitude, &state->rates);
    // dt*dq = T * quaternion_derivative(q, w)
    quaternion_t dt_dq = quaternion_scale(dt, &dq);
    // q_new = q + T * quaternion_derivative(q, w);
    quaternion_t q_new = quaternion_add(&(state->attitude), &dt_dq);
    state_new.attitude = quaternion_normalize(&q_new);

    // rate update
    vector3d_t J_times_omega = math_vector3d_rotate(&inertia_matrix, &state->rates); // param.J*w
    vector3d_t gyro_moment =
        math_vector3d_cross(&state->rates, &J_times_omega); // cross(w, param.J*w)
    vector3d_t moment = math_vector3d_subt(&torque, &gyro_moment); // torque - cross(w, param.J*w)
    vector3d_t omega_dot = math_vector3d_rotate(
        &inertia_matrix_inv, &moment
    ); // inv(param.J) * (torque - cross(w, param.J*w))
    vector3d_t domega =
        math_vector3d_scale(dt, &omega_dot); // T * inv(param.J) * (torque - cross(w, param.J*w))

    vector3d_t omega_new = math_vector3d_add(&state->rates, &domega);
    state_new.rates = omega_new;

    // velocity update
    vector3d_t acceleration_transport =
        math_vector3d_cross(&state->rates, &state->velocity); // cross(w,v)
    vector3d_t acceleration_body =
        math_vector3d_subt(&(input->acceleration), &acceleration_transport); // a - cross(w,v)
    vector3d_t acceleration_gravity = math_vector3d_rotate(&S, &grav_acc); // S*param.g
    vector3d_t acceleration_true =
        math_vector3d_add(&acceleration_body, &acceleration_gravity); // a - cross(w,v) + S*param.g
    vector3d_t dvelocity =
        math_vector3d_scale(dt, &acceleration_true); // T * (a - cross(w,v) + S*param.g)
    vector3d_t v_new =
        math_vector3d_add(&(state->velocity), &dvelocity); // v + T * (a - cross(w,v) + S*param.g)
    state_new.velocity = v_new;

    // altitude update
    vector3d_t v_earth = math_vector3d_rotate(&ST, &(state->velocity)); // (S')*v
    state_new.altitude = state->altitude + dt * v_earth.x;

    // canard coeff derivative
    float CL_new = state->CL + dt * (-1 / tau_cl_alpha * (state->CL - cl_alpha));
    state_new.CL = CL_new;

    // actuator dynamics
    // linear 1st order
    float delta_new = state->delta + dt * (-1 / tau * (state->delta - input->cmd));
    state_new.delta = delta_new;

    return state_new;
}

// void model_dynamics_jacobian(arm_matrix_instance_f32 *dynamics_jacobian, x_state_t *state,
// u_dynamics_t *input, uint32_t timestamp)