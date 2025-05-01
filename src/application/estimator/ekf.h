/**
 * Extended Kalman Filter
 */
#ifndef EKF_H
#define EKF_H

#include "application/estimator/estimator_types.h"
#include "arm_math.h"
#include "common/math/math.h"
#include <stdbool.h>

/**
 * @brief helper function to initialize matrices
 * @callergraph ekf_algorithm
 *
 * @param dt to scale matrix Q beforehand
 * @remark specific matrices check the .c file
 */
void ekf_init(double dt);

/**
 * @brief Extended Kalman Filter algorithm
 * @callgraph ekf_init, ekf_predict, ekf_correct
 *
 * @param x_state_t current state pointer to be altered
 * @param arm_matrix_instance_f64 P pointer to store new covariance
 * @param u_dynamics_t input signal pointer
 * @param y_imu_t MTI and ALTIMU: imu measurement and bias
 * @param double encoder
 * @param double dt time step
 * @param bool whether imus are dead
 */
void ekf_algorithm(
    x_state_t *state, arm_matrix_instance_f64 *P, const u_dynamics_t *input, const y_imu_t *imu_mti,
    const y_imu_t *bias_mti, const y_imu_t *imu_altimu, const y_imu_t *bias_altimu, double encoder,
    double dt, const bool is_dead_MTI, const bool is_dead_ALTIMU
);

// EKF PREDICTION STEP FUNC

/**
 * @brief a-priori state and covariance estimates
 * @callergraph ekf algorithm
 *
 * @param x_state_t state pointer to new state to be altered
 * @param arm_matrix_instance_f64 P pointer to store new covariance
 * @param u_dynamics_t input imu inputs
 * @param double dt time step
 */
void ekf_matrix_predict(
    x_state_t *state, arm_matrix_instance_f64 *P, const u_dynamics_t *input, double dt
);

/**
 * @brief computes a-posteriori state and covariance estimates
 * @callergraph ekf algorithm
 *
 * @param x_state_t state pointer to new state to be altered
 * @param arm_matrix_instance_f64 P pointer to store new covariance
 * @param arm_matrix_instance_f64 R pointer to the diag matrices
 * @param uint16_t size_measurement
 * @param y_imu_t imu measurement and bias
 */
void ekf_matrix_correct(
    x_state_t *state, arm_matrix_instance_f64 *P, const arm_matrix_instance_f64 *R,
    const uint16_t size_measurement, const y_imu_t *imu, const y_imu_t *bias
);

#endif