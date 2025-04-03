#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/model/model_airdata.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// parameters go here as static const, or as define
static const float c_canard =
    (0.02) * (8 * 0.0254 / 2 + 0.05); // area of canard * lever arm of canard to x-axis
static const float cn_alpha = 5.0f; // pitch forcing coeff
static const float cn_omega = 0.0f; // roll damping coeff
static const float area_reference = M_PI * pow((8 * 0.0254 / 2), 2); // cross section of body tube
static const float c_aero =
    area_reference * (-0.5f); // area_reference * (length_cp-length_cg), center of pressure(cp):
                              // -0.5, center of gravity(cg): 0
static const matrix3d_t inertia_matrix_inv = {
    .array = {{1 / 0.225, 0, 0}, {0, 1 / 52.0, 0}, {0, 0, 1 / 52.0}}
}; // inertia matrix of the rocket (J in matlab)
static vector3d_t grav_acc = {
    .array = {-9.8, 0, 0}
}; // gravitational acceleration in the geographic inertial frame (g in matlab)
static const float tau_cl_alpha = 2.0f; // time constant to converge Cl back to 1.5 in filter
static const float cl_alpha = 1.5f; // estimated coefficient of lift, const with Ma
static const float tau = 1 / 40.0f; // time constant of first order actuator dynamics

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
    vector3d_t vector_helper1 = {.array = {1, 0, 0}};
    vector3d_t vector_helper2 = {.array = {0, sin_alpha, -sin_beta}};
    vector3d_t vector_helper3 = {.array = {0, est_state.rates.array[1], est_state.rates.array[2]}};

    vector3d_t torque_canards =
        math_vector3d_scale(est_state.CL * est_state.delta * c_canard * p_dyn, &vector_helper1);

    vector3d_t torque_aero = math_vector3d_scale(
        p_dyn * c_aero,
        math_vector3d_add(
            &math_vector3d_scale(cn_alpha, &vector_helper2),
            &math_vector3d_scale(cn_omega, &vector_helper3)
        )
    );
    vector3d_t torque = math_vector3d_add(&torque_canards, &torque_aero);

    // update attitude quaternion
    quaternion_t q_new =
        state.attitude + dt * quaternion_derivative(&est_state.attitude, &est_state.rates);
    state_new.attitude = quaternion_normalize(&q_new);

    // rate update: missing matrix inverse
    vector3d_t vector_helper4 = math_vector3d_subt(
        &torque,
        &math_vector3d_cross(
            &est_state->rates, math_vector3d_rotate(&inertia_matrix, &est_state->rates)
        )
    ); // torque - cross(w, param.J*w)
    vector3d_t w_new = math_vector3d_add(
        &est_state->rates,
        &math_vector3d_scale(dt, &math_vector3d_rotate(&inertia_matrix_inv, &vector_helper4))
    );
    state_new.rates = w_new;

    // velocity update
    vector3d_t vector_helper5 = math_vector3d_add(
        &math_vector3d_subt(
            &est_input.acceleration, &math_vector3d_cross(&est_state->rates, &est_state->velocity)
        ),
        &math_vector3d_rotate(&S, &grav_acc)
    ); // a - cross(w,v) + S*param.g
    vector3d_t v_new =
        math_vector3d_add(est_state->velocity, &math_vector3d_scale(dt, &vector_helper5));
    state_new.velocity = v_new;

    // altitude update
    vector3d_t v_earth = math_vector3d_rotate(&math_matrix3d_transp(&S), &est_state.velocity);
    state_new.altitude = est_state->altitude + dt * v_earth.array[0];

    // canard coeff derivative
    float CL_new = est_state->CL + dt * (-1 / tau_cl_alpha * (est_state->CL - cl_alpha));
    new_state.CL = CL_new;

    // actuator dynamics
    // linear 1st order
    float delta_new =
        est_state->delta + dt * (-1 / tau * (est_state->delta - est_input->canard_command));
    state_new.delta = delta_new;

    return state_new;
}