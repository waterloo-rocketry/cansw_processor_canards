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
 * @param state current state pointer to be altered
 * @param P_flat P pointer to store new covariance, to help build arm_matrix_instance_f64
 * @param input input signal pointer
 * @param imu_mti pololu imu measurement
 * @param bias_mti pololu bias
 * @param imu_altimu movella imu measurement
 * @param bias_altimu movella bias
 * @param encoder encoder measurement
 * @param dt time step
 * @param is_dead_MTI true if movella is dead
 * @param is_dead_ALTIMU true if pololu is dead
 */
void ekf_algorithm(
    x_state_t *x_state, double P_flat[SIZE_STATE * SIZE_STATE], const y_imu_t *imu_mti,
    const y_imu_t *bias_mti, const y_imu_t *imu_altimu, const y_imu_t *bias_altimu, double cmd,
    double encoder, double dt, const bool is_dead_MTI, const bool is_dead_ALTIMU
);

// EKF PREDICTION STEP FUNC

/**
 * @brief a-priori state and covariance estimates
 * @callergraph ekf algorithm
 *
 * @param x_state state pointer to new state to be altered
 * @param P_flat P pointer to store new covariance
 * @param u_input input imu inputs
 * @param Q covariance matrix
 * @param dt time step
 */
void ekf_matrix_predict(
    x_state_t *x_state, double P_flat[SIZE_STATE * SIZE_STATE], const u_dynamics_t *u_input,
    double Q[SIZE_STATE * SIZE_STATE], double dt
);

/**
 * @brief computes a-posteriori state and covariance estimates
 * @callergraph ekf algorithm
 *
 * @param x_state state pointer to new state to be altered
 * @param P_flat P pointer to store new covariance
 * @param R pointer to the diag matrices
 * @param y_meas the measurement, such as IMU_1(4:end) or encoder
 * @param bias measurement bias (imu or encoder)
 *
 * for now, hard-coded to do imu bias (no encoder allowed)
 */
void ekf_matrix_correct(
    x_state_t *x_state, double P_flat[SIZE_STATE * SIZE_STATE], const arm_matrix_instance_f64 *R,
    const y_imu_t *y_meas_full, const y_imu_t *bias
);

#endif