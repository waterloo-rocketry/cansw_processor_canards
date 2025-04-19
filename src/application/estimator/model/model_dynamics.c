#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/model/model_aerodynamics.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include <math.h>

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

// jacobians flattened array
// flattened array for arm_matrix_instance_f64
static float64_t pData[X_STATE_SIZE_ITEMS * X_STATE_SIZE_ITEMS] = {0};

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

    // forces and torque func -- see model_aerodynamics
    vector3d_t torque;
    aerodynamics(state, &airdata, &torque);

    // update attitude quaternion
    state_new.attitude = quaternion_update(&state->attitude, &state->rates, dt);

    // rate update
    const vector3d_t J_times_omega = math_vector3d_rotate(&J, &state->rates); // param.J*w
    const vector3d_t gyro_moment =
        math_vector3d_cross(&state->rates, &J_times_omega); // cross(w, param.J*w)
    const vector3d_t moment =
        math_vector3d_subt(&torque, &gyro_moment); // torque - cross(w, param.J*w)
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
    state_new.CL = state->CL + dt * (1 / tau_cl_alpha * (Cl_theory - state->CL));

    // actuator dynamics
    // linear 1st order
    const double delta_new = state->delta + dt * (1 / tau * (input->cmd - state->delta));
    state_new.delta = delta_new;

    return state_new;
}

/**
 * @brief helper function to construct pData
 * @param start_coor_x starting coordinate x in 13x13 jacobian (0-indexed)
 * @param start_coor_y starting coordinate y in 13x13 jacobian (0-indexed)
 * @param num_row number of rows of sub-structure
 * @param num_col number of columns of sub-structure
 * @param flat_data pointer to the data of sub-structure in row major order
 *
 * 2D coor (x, y) flattening to 1D coor (index): index = x * num_col + y
 */
static void
write_pData(int start_coor_x, int start_coor_y, int num_row, int num_col, double *flat_data) {
    for (int i = 0; i < num_row; i++) {
        for (int j = 0; j < num_col; j++) {
            pData[(start_coor_x + i) * X_STATE_SIZE_ITEMS + (start_coor_y + j)] =
                flat_data[i * num_col + j];
        }
    }
}

void model_dynamics_jacobian(
     arm_matrix_instance_f64 *dynamics_jacobian, const x_state_t *state,
    const u_dynamics_t *input, double dt
) {
    // airdata calc
    estimator_airdata_t airdata = model_airdata(state->altitude);

    // quaternion attitude rows (q, 1:4)
    double q_q[4*4];
    double q_w[4*3];
    quaternion_update_jacobian(&q_q[0], &q_w[0], &(state->attitude), &(state->rates), dt);
    // write to pData
    write_pData(0, 0, 4, 4, &q_q[0]); // J_x(1:4,1:4) = q_q; % column q (attitude)
    write_pData(0, 4, 4, 3, &q_w[0]); // J_x(1:4, 5:7) = q_w; % column w (rates)

    // angular rate rows (w, 5:7)
    // **aerodynamics_jacobian start
    const vector3d_t helper_vx =
        math_vector3d_scale(state->CL * state->delta * c_canard * airdata.density, &state->velocity);
    const matrix3d_t torque_vx = {
        .array = {helper_vx.x, helper_vx.y, helper_vx.z, 0, 0, 0, 0, 0, 0}
    };

    const vector3d_t helper_vyz = math_vector3d_scale(0.5 * c_aero * cn_alpha * airdata.density, &state->velocity);
    const matrix3d_t torque_vyz = {
        .array = {0, 0, 0, helper_vyz.z, 0, helper_vyz.x, -helper_vyz.y, -helper_vyz.x, 0}
    };

    const matrix3d_t torque_v = math_matrix3d_add(&torque_vx, &torque_vyz);

    const double dyn_pressure = 0.5 * airdata.density * pow(quaternion_norm((quaternion_t *)&(state->velocity)), 2);
    const vector3d_t torque_cl = {.array = {state->delta * c_canard * dyn_pressure, 0, 0}};
    const vector3d_t torque_delta = {.array = {state->CL * c_canard * dyn_pressure, 0, 0}};
    // **aerodynamics_jacobian end
    // **tilde start
    const matrix3d_t w_tilde = {.array = {0, -state->rates.z, state->rates.y, state->rates.z, 0, -state->rates.x, -state->rates.y, state->rates.x, 0}}; 
    // **tilde end
    const matrix3d_t J_w_tilde = math_matrix3d_mult(&J, &w_tilde); // J * w_tilde

    // update jacobian output
    arm_mat_init_f64(dynamics_jacobian, X_STATE_SIZE_ITEMS, X_STATE_SIZE_ITEMS, &pData[0]);
}