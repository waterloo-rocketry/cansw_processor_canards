#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_aerodynamics.h"
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

// mass and inertia
static const matrix3d_t J = {
    .array = {{0.17, 0, 0}, {0, 11.0, 0}, {0, 0, 11.0}}
}; // inertia matrix of the rocket
static const matrix3d_t J_inv = {
    .array = {{1 / 0.17, 0, 0}, {0, 1 / 11.0, 0}, {0, 0, 1 / 11.0}}
}; // inverse of inertia matrix of the rocket (J^-1 in matlab)
static vector3d_t g = {
    .array = {-9.81, 0, 0}
}; // gravitational acceleration in the geographic inertial frame 

// airfoil
static const double tau_cl_alpha =
    5; // time constant to converge Cl back to theoretical value in filter
static const double tau = 1 / 20.0; // time constant of first order actuator dynamics

// helper functions

static inline double ms_to_seconds(double x) {
    return x / 1000.0;
} // convert milliseconds to seconds
static inline double sign(double x) {
    return (x) > 0 ? 1 : ((x) < 0 ? -1 : 0);
} // returns the sign of x; pos: 1, neg: -1, zero: 0

/*
 * Dynamics update
 * returns the new integrated state
 */
x_state_t model_dynamics_update(const x_state_t *state, const u_dynamics_t *input, double dt) {
    x_state_t state_new;

    // Compute rotation matrix from attitude quaternion

    const matrix3d_t S = quaternion_rotmatrix(&(state->attitude));

    const matrix3d_t ST = math_matrix3d_transp(&S); // S'

    /*
     * Aerodynamics
     */
    // get current air information (density is needed for aerodynamics)
    const estimator_airdata_t airdata = model_airdata(state->altitude);

    // forces and torque func: from restructuring
    const vector3d_t *torque = aerodynamics(state, &airdata);

    // update attitude quaternion
    // dq = quaternion_derivative(q, w)
    const quaternion_t dq = quaternion_derivative(&state->attitude, &state->rates);
    // dt*dq = T * quaternion_derivative(q, w)
    const quaternion_t dt_dq = quaternion_scale(dt, &dq);
    // q_new = q + T * quaternion_derivative(q, w);
    const quaternion_t q_new = quaternion_add(&(state->attitude), &dt_dq);
    state_new.attitude = quaternion_normalize(&q_new);

    // rate update
    const vector3d_t J_times_omega = math_vector3d_rotate(&J, &state->rates); // param.J*w
    const vector3d_t gyro_moment =
        math_vector3d_cross(&state->rates, &J_times_omega); // cross(w, param.J*w)
    const vector3d_t moment =
        math_vector3d_subt(torque, &gyro_moment); // torque - cross(w, param.J*w)
    const vector3d_t omega_dot =
        math_vector3d_rotate(&J_inv, &moment); // inv(param.J) * (torque - cross(w, param.J*w))
    const vector3d_t domega =
        math_vector3d_scale(dt, &omega_dot); // T * inv(param.J) * (torque - cross(w, param.J*w))

    const vector3d_t omega_new = math_vector3d_add(&state->rates, &domega);
    state_new.rates = omega_new;

    // velocity update
    const vector3d_t acceleration_transport =
        math_vector3d_cross(&state->rates, &state->velocity); // cross(w,v)
    const vector3d_t acceleration_body =
        math_vector3d_subt(&(input->acceleration), &acceleration_transport); // a - cross(w,v)
    const vector3d_t acceleration_gravity = math_vector3d_rotate(&S, &g); // S*param.g
    const vector3d_t acceleration_true =
        math_vector3d_add(&acceleration_body, &acceleration_gravity); // a - cross(w,v) + S*param.g
    const vector3d_t dvelocity =
        math_vector3d_scale(dt, &acceleration_true); // T * (a - cross(w,v) + S*param.g)
    const vector3d_t v_new =
        math_vector3d_add(&(state->velocity), &dvelocity); // v + T * (a - cross(w,v) + S*param.g)
    state_new.velocity = v_new;

    // altitude update
    const vector3d_t v_earth = math_vector3d_rotate(&ST, &(state->velocity)); // (S')*v
    state_new.altitude = state->altitude + dt * v_earth.x;

    // canard coeff derivative
    // returns Cl to expected value slowly, to force convergence in EKF
    const double mach_num =
        math_vector3d_norm(&state->velocity) / airdata.mach_local; // norm(v) / mach_local
    const double Cl_theory = airfoil(mach_num); // see model_aerodynamics for function def
    state_new.CL = state->CL + dt * (-1 / tau_cl_alpha * (Cl_theory - state->CL));

    // actuator dynamics
    // linear 1st order
    const double delta_new = state->delta + dt * (-1 / tau * (input->cmd - state->delta));
    state_new.delta = delta_new;

    return state_new;
}

// void model_dynamics_jacobian(const arm_matrix_instance_f32 *dynamics_jacobian, const x_state_t
// *state, const u_dynamics_t *input, double dt)