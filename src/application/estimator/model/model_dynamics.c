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
static const double c_canard =
    (2 * (4 * 0.0254) * (2.5 * 0.0254)) * (0.203 / 2 + 0.0254); // moment arm * area of canard
static const double cn_alpha = 5.0; // pitch forcing coeff
static const double cn_omega = 0.0; // roll damping coeff
static const double area_reference = M_PI * pow((0.203 / 2), 2); // cross section of body tube
// c_aero = area_reference * (length_cp-length_cg), center of pressure(cp): -0.5, center of
// gravity(cg): 0
static const double c_aero = area_reference * (-0.5);

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
static const double tau = 1 / 20.0; // time constant of first order actuator dynamics
static const double canard_sweep = 60.0 / 180 * M_PI; // 60 degrees in radians

// helper functions
static inline double cot(double x){return 1.0/tan(x);}
static inline double ms_to_seconds(double x){return x / 1000.0;} // convert milliseconds to seconds
static inline double sign(double x){return (x) > 0 ? 1 : ((x) < 0 ? -1 : 0);} // returns the sign of x; pos: 1, neg: -1, zero: 0


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

    const double p_dyn = airdata.density / 2.0 * pow(math_vector3d_norm(&(state->velocity)), 2);
    const double mach_num =
        math_vector3d_norm(&state->velocity) / airdata.mach_local; // norm(v) / mach_local

    double sin_alpha = 0.0, sin_beta = 0.0;
    // angle of attack/sideslip
    if (abs(state->velocity.x) >= 0.5) {
        sin_alpha = (state->velocity.z / state->velocity.x) /
                    sqrt(pow(state->velocity.z, 2) / pow(state->velocity.x, 2) + 1);
        sin_beta = (state->velocity.y / state->velocity.x) /
                   sqrt(pow(state->velocity.y, 2) / pow(state->velocity.x, 2) + 1);
    } else {
        sin_alpha = sign(state->velocity.z);
        sin_beta = sign(state->velocity.y);
    }

    // torque calculations
    const vector3d_t torque_unit_x = {.array = {1, 0, 0}};
    const vector3d_t torque_sin_yz = {.array = {0, cn_alpha * sin_alpha, -cn_alpha * sin_beta}};
    // param.Cn_alpha*[0; sin_alpha; -sin_beta]
    const vector3d_t torque_omega_yz = {
        .array = {0, cn_omega * state->rates.y, cn_omega * state->rates.z}
    }; // param.Cn_omega*[0; w(2); w(3)]
    const vector3d_t torque_yz = math_vector3d_add(
        &torque_sin_yz,
        &torque_omega_yz
    ); // param.Cn_alpha*[0; sin_alpha; -sin_beta] + param.Cn_omega*[0; w(2); w(3)]
    const vector3d_t torque_canards =
        math_vector3d_scale(state->CL * state->delta * c_canard * p_dyn, &torque_unit_x);
    const vector3d_t torque_aero = math_vector3d_scale(p_dyn * c_aero, &torque_yz);
    const vector3d_t torque = math_vector3d_add(&torque_canards, &torque_aero);

    // update attitude quaternion
    // dq = quaternion_derivative(q, w)
    const quaternion_t dq = quaternion_derivative(&state->attitude, &state->rates);
    // dt*dq = T * quaternion_derivative(q, w)
    const quaternion_t dt_dq = quaternion_scale(dt, &dq);
    // q_new = q + T * quaternion_derivative(q, w);
    const quaternion_t q_new = quaternion_add(&(state->attitude), &dt_dq);
    state_new.attitude = quaternion_normalize(&q_new);

    // rate update
    const vector3d_t J_times_omega =
        math_vector3d_rotate(&inertia_matrix, &state->rates); // param.J*w
    const vector3d_t gyro_moment =
        math_vector3d_cross(&state->rates, &J_times_omega); // cross(w, param.J*w)
    const vector3d_t moment =
        math_vector3d_subt(&torque, &gyro_moment); // torque - cross(w, param.J*w)
    const vector3d_t omega_dot = math_vector3d_rotate(
        &inertia_matrix_inv, &moment
    ); // inv(param.J) * (torque - cross(w, param.J*w))
    const vector3d_t domega =
        math_vector3d_scale(dt, &omega_dot); // T * inv(param.J) * (torque - cross(w, param.J*w))

    const vector3d_t omega_new = math_vector3d_add(&state->rates, &domega);
    state_new.rates = omega_new;

    // velocity update
    const vector3d_t acceleration_transport =
        math_vector3d_cross(&state->rates, &state->velocity); // cross(w,v)
    const vector3d_t acceleration_body =
        math_vector3d_subt(&(input->acceleration), &acceleration_transport); // a - cross(w,v)
    const vector3d_t acceleration_gravity = math_vector3d_rotate(&S, &grav_acc); // S*param.g
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
    // double CL_new = state->CL + dt * (-1 / tau_cl_alpha * (state->CL - cl_alpha));
    double cone = 0;
    if (mach_num <= 1) {
        cone = 0;
    } else {
        cone = acos(1 / mach_num);
    }
    if (cone > canard_sweep) {
        state_new.CL = 4 / sqrt(pow(mach_num, 2) - 1); // 4 / sqrt(mach_num^2 - 1)
    } else {
        const double m = cot(canard_sweep) / cot(cone);
        const double a = m * (0.38 + 2.26 * m - 0.86 * m * m); // m*(0.38+2.26*m-0.86*m^2)
        state_new.CL = 2 * M_PI * M_PI * cot(canard_sweep) /
                       (M_PI + a); // 2*pi^2*cot(param.canard_sweep) / (pi + a)
    }

    // actuator dynamics
    // linear 1st order
    const double delta_new = state->delta + dt * (-1 / tau * (state->delta - input->cmd));
    state_new.delta = delta_new;

    return state_new;
}

// void model_dynamics_jacobian(const arm_matrix_instance_f32 *dynamics_jacobian, const x_state_t *state,
// const u_dynamics_t *input, double dt)