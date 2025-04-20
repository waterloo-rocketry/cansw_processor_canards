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

// AERODYNAMIC PARAMETERS
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

// AIRFOIL
static const double tau_cl_alpha =
    5; // time constant to converge Cl back to theoretical value in filter
static const double tau = 1 / 20.0; // time constant of first order actuator dynamics

// JACOBIANS
/**
 * @brief constants defined for the sizes of structures
 * @example rotation matrix 3x4: SIZE_VECTOR * SIZE_QUAT
 * @example quaternion update matrix q_q 4x4: SIZE_QUAT * SIZE_QUAT
 * @example quaternion update matrix q_w 4x3: SIZE_QUAT * SIZE_VECTOR
 * @example matrix3d_t 3x3: SIDE_MATRIX * SIDE_MATRIX
 * @example vector3d_t 3x1: SIZE_VECTOR * SIZE_1D
 * @example final jacobian matrix 13x13: X_STATE_SIZE_ITEMS * X_STATE_SIZE_ITEMS
 */
#define SIZE_1D 1
#define SIDE_MATRIX 3

// flattened array for arm_matrix_instance_f64
static float64_t pData[X_STATE_SIZE_ITEMS * X_STATE_SIZE_ITEMS] = {0};

// MATRIX DEFS
// rotation matrix
typedef union {
    double flat[SIZE_VECTOR * SIZE_QUAT];
    struct {
        double s11, s12, s13, s14;
        double s21, s22, s23, s24;
        double s31, s32, s33, s34;
    };
} rotation_jacobian_t;
// quaternion update 4x4 matrix
typedef union {
    double flat[SIZE_QUAT * SIZE_QUAT];
    struct {
        double q11, q12, q13, q14;
        double q21, q22, q23, q24;
        double q31, q32, q33, q34;
        double q41, q42, q43, q44;
    };
} quaternion_update_matrix_q_t;
// quaternion update 4x3 matrix
typedef union {
    double flat[SIZE_QUAT * SIZE_VECTOR];
    struct {
        double q11, q12, q13;
        double q21, q22, q23;
        double q31, q32, q33;
        double q41, q42, q43;
    };
} quaternion_update_matrix_w_t;

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
 * @brief helper function to construct pData for jacobian matrix instance
 * @param start_coor_x starting coordinate x in 13x13 jacobian (0-indexed)
 * @param start_coor_y starting coordinate y in 13x13 jacobian (0-indexed)
 * @param num_row number of rows of sub-structure
 * @param num_col number of columns of sub-structure
 * @param flat_data pointer to the data of sub-structure in row major order
 *
 * 2D coor (x, y) flattening to 1D coor (index): index = x * num_col + y
 */
static void
write_pData(int start_coor_x, int start_coor_y, int num_row, int num_col, const double *flat_data) {
    for (int i = 0; i < num_row; i++) {
        for (int j = 0; j < num_col; j++) {
            pData[(start_coor_x + i) * X_STATE_SIZE_ITEMS + (start_coor_y + j)] =
                flat_data[i * num_col + j];
        }
    }
}

void model_dynamics_jacobian(
    arm_matrix_instance_f64 *dynamics_jacobian, const x_state_t *state, const u_dynamics_t *input,
    double dt
) {
    /**
     * airdata calc
     */
    estimator_airdata_t airdata = model_airdata(state->altitude);

    /**
     * quaternion attitude rows (q, 1:4)
     */
    quaternion_update_matrix_q_t q_q = {0}; // 4x4
    quaternion_update_matrix_w_t q_w = {0}; // 4x3
    quaternion_update_jacobian(&q_q.flat[0], &q_w.flat[0], &(state->attitude), &(state->rates), dt);
    // write to pData: a matrix 4x4 and a matrix 4x3
    write_pData(
        0, 0, SIZE_QUAT, SIZE_QUAT, &q_q.flat[0]
    ); // J_x(1:4,1:4) = q_q; % column q (attitude)
    write_pData(
        0, 4, SIZE_QUAT, SIZE_VECTOR, &q_w.flat[0]
    ); // J_x(1:4, 5:7) = q_w; % column w (rates)

    /**
     * angular rate rows (w, 5:7)
     */
    // **aerodynamics_jacobian start
    const vector3d_t helper_vx = math_vector3d_scale(
        state->CL * state->delta * c_canard * airdata.density, &state->velocity
    );
    const matrix3d_t torque_vx = {
        .array = {{helper_vx.x, helper_vx.y, helper_vx.z}, {0, 0, 0}, {0, 0, 0}}
    };

    const vector3d_t helper_vyz =
        math_vector3d_scale(0.5 * c_aero * cn_alpha * airdata.density, &state->velocity);
    const matrix3d_t torque_vyz = {
        .array = {{0, 0, 0}, {helper_vyz.z, 0, helper_vyz.x}, {-helper_vyz.y, -helper_vyz.x, 0}}
    };

    const matrix3d_t torque_v = math_matrix3d_add(&torque_vx, &torque_vyz);

    const double dyn_pressure = 0.5 * airdata.density *
                                pow(math_vector3d_norm((vector3d_t *)&(state->velocity)),
                                    2); // 0.5 * airdata.density * norm(v)^2
    const vector3d_t torque_cl = {.array = {state->delta * c_canard * dyn_pressure, 0, 0}};
    const vector3d_t torque_delta = {.array = {state->CL * c_canard * dyn_pressure, 0, 0}};
    // **aerodynamics_jacobian end
    // **tilde start
    const matrix3d_t w_tilde = {
        .array = {
            {0, state->rates.z, -state->rates.y},
            {-state->rates.z, 0, state->rates.x},
            {state->rates.y, -state->rates.x, 0}
        }
    }; // -tilde(w)
    // **tilde end
    const matrix3d_t J_w_tilde = math_matrix3d_mult(&J, &w_tilde); // -param.J * tilde(w)
    const matrix3d_t J_inv_scaled = {
        .array = {{1 / 0.17 * (dt), 0, 0}, {0, 1 / 11.0 * (dt), 0}, {0, 0, 1 / 11.0 * (dt)}}
    }; // dt * param.Jinv
    const matrix3d_t J_inv_torque =
        math_matrix3d_mult(&J_inv_scaled, &J_w_tilde); // dt * param.Jinv * (- param.J*tilde(w))
    const matrix3d_t idn = {.array = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}; // identity matrix
    const matrix3d_t w_w =
        math_matrix3d_add(&idn, &J_inv_torque); // eye(3) + dt * param.Jinv * (- param.J*tilde(w))
    const matrix3d_t w_v =
        math_matrix3d_mult(&J_inv_scaled, &torque_v); // dt * param.Jinv * torque_v
    const vector3d_t w_cl =
        math_vector3d_rotate(&J_inv_scaled, &torque_cl); // dt * param.Jinv * torque_cl
    const vector3d_t w_delta =
        math_vector3d_rotate(&J_inv_scaled, &torque_delta); // dt * param.Jinv * torque_delta

    // write to pData: 2 matrices 2 vectors
    write_pData(4, 4, SIDE_MATRIX, SIDE_MATRIX, &w_w.flat[0]); // J_x(5:7,5:7) = w_w; % column w
    write_pData(4, 7, SIDE_MATRIX, SIDE_MATRIX, &w_v.flat[0]); // J_x(5:7,8:10) = w_v; % column v
    write_pData(4, 11, SIZE_VECTOR, SIZE_1D, &w_cl.array[0]); // J_x(5:7,12) = w_cl; % column Cl
    write_pData(
        4, 12, SIZE_VECTOR, SIZE_1D, &w_delta.array[0]
    ); // J_x(5:7,13) = w_delta; % column delta

    /**
     * velocity rows (v, 8:10)
     */
    rotation_jacobian_t v_q = {0}; // 3x4
    quaternion_rotate_jacobian(&v_q.flat[0], &(state->attitude), &g);
    for (int i = 0; i < (SIZE_VECTOR * SIZE_QUAT); i++) {
        v_q.flat[i] *= dt;
    } // dt * quaternion_rotate_jacobian(q, param.g)

    const vector3d_t v_scaled = math_vector3d_scale(-dt, &state->velocity);
    // **tilde start
    const matrix3d_t v_w = {
        .array = {
            {0, -v_scaled.z, v_scaled.y}, {v_scaled.z, 0, -v_scaled.x}, {-v_scaled.y, v_scaled.x, 0}
        }
    }; // - dt * tilde(v)
    // **tilde end

    const vector3d_t w_scaled = math_vector3d_scale(dt, &state->rates);
    // **tilde start
    const matrix3d_t w_scaled_tilde = {
        .array = {
            {0, -w_scaled.z, w_scaled.y}, {w_scaled.z, 0, -w_scaled.x}, {-w_scaled.y, w_scaled.x, 0}
        }
    }; // dt * tilde(w)
    // **tilde end
    const matrix3d_t v_v = math_matrix3d_add(&idn, &w_scaled_tilde); // eye(3) + dt * tilde(v)
    // write to pData: a 3x4 matrix, 2 regular matrix3d_t
    write_pData(7, 0, SIZE_VECTOR, SIZE_QUAT, &v_q.flat[0]); // J_x(8:10,1:4) = v_q; % column q
    write_pData(7, 4, SIDE_MATRIX, SIDE_MATRIX, &v_w.flat[0]); // J_x(8:10,5:7) = v_w; % column w
    write_pData(7, 7, SIDE_MATRIX, SIDE_MATRIX, &v_v.flat[0]); //  J_x(8:10,8:10) = v_v; % column v

    /**
     * altitude rows (alt, 11)
     */
    const quaternion_t q_inv = quaternion_inverse(&(state->attitude));
    rotation_jacobian_t r_q = {0}; // 3x4
    quaternion_rotate_jacobian(
        &r_q.flat[0], &q_inv, &(state->velocity)
    ); // quaternion_rotate_jacobian(quaternion_inv(q), v)
    for (int i = 0; i < (SIZE_VECTOR * SIZE_QUAT); i++) {
        r_q.flat[i] *= dt;
    } // dt * quaternion_rotate_jacobian(q_inv, v)

    const double alt_q[SIZE_QUAT] = {r_q.s11, r_q.s12, r_q.s13, r_q.s14}; // alt_q = r_q(1,:)

    matrix3d_t r_v = quaternion_rotmatrix(&q_inv);
    for (int i = 0; i < (SIDE_MATRIX * SIDE_MATRIX); i++) { // we dont have matrix scale anymore :(
        r_v.flat[i] *= dt;
    } // dt * quaternion_rotmatrix(q_inv)

    const vector3d_t alt_v = {.array = {r_v.s11, r_v.s12, r_v.s13}}; // alt_v = r_v(1,:)

    const double alt_alt = 1;

    // write to pData: a 1x4 vector, a vector, a scalar
    write_pData(10, 0, SIZE_1D, SIZE_QUAT, &alt_q[0]); // J_x(11,1:4) = alt_q; % column q
    write_pData(10, 7, SIZE_1D, SIZE_VECTOR, &alt_v.array[0]); // J_x(11,8:10) = alt_v; % column v
    write_pData(10, 10, SIZE_1D, SIZE_1D, &alt_alt); // J_x(11, 11) = alt_alt; % column alt

    /**
     * coefficient row (Cl, 12)
     */
    const double Cl_Cl = 1 - dt * 1 / tau_cl_alpha;
    // write to pData: a scalar
    write_pData(11, 11, SIZE_1D, SIZE_1D, &Cl_Cl); // J_x(12,12) = Cl_cl; % column Cl

    /**
     * canard angle row (delta, 13)
     */
    const double delta_delta = 1 - dt * 1 / tau; // delta_delta = 1 - dt * 1/param.tau
    // write to pData: a scalar
    write_pData(12, 12, SIZE_1D, SIZE_1D, &delta_delta); // J_x(13,13) = delta_delta; % column delta

    /**
     * update jacobian output
     */
    arm_mat_init_f64(dynamics_jacobian, X_STATE_SIZE_ITEMS, X_STATE_SIZE_ITEMS, &pData[0]);
}