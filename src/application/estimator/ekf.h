/**
 * Extended Kalman Filter
 */
#ifndef EKF_H
#define EKF_H

#include "application/estimator/estimator_types.h"
#include "arm_math.h"
#include "common/math/math.h"

/**
 * @brief helper function to initialize matrices 
 * @param dt to scale matrices beforehand
 * @remark for specific matrices see .c file 
 */
void ekf_init(double dt);

/**
 * @brief Extended Kalman Filter algorithm
 * @callgraph ekf_init, ekf_predict, ekf_correct
 */
void ekf_algorithm(
    x_state_t *state, const u_dynamics_t *input, const y_imu_t *imu_mti, const y_imu_t *bias_mti,
    const y_imu_t *imu_altimu, const y_imu_t *bias_altimu, const float *encoder, double dt
);

// EKF PREDICTION STEP FUNC

/**
 * @callergraph computes a-priori state and covariance estimates
 * @param x_state_t pointer to new state
 * @param P_new pointer to new covariance
 * @param state current state
 * @param input imu inputs
 * @param dt time step 
 */
void ekf_matrix_predict(
    x_state_t *x_new, arm_matrix_instance_f64 *P_new, const x_state_t *state,
    const u_dynamics_t *input, double dt
);

/**
 * @callergraph computes a-posteriori state and covariance estimates
 * 
 */
void ekf_matrix_correct(
    arm_matrix_instance_f32 *P, arm_matrix_instance_f32 *K, const arm_matrix_instance_f32 *H,
    const arm_matrix_instance_f32 *R, const uint16_t size_measurement
);

#endif