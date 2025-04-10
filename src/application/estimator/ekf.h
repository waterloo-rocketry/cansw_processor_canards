/**
 * Extended Kalman Filter
 */
#ifndef EKF_H
#define EKF_H

#include "application/estimator/estimator_types.h"
#include "arm_math.h"
#include "common/math/math.h"

void ekf_init(void *arg);
void ekf_algorithm(
    x_state_t *state, const u_dynamics_t *input, const y_imu_t *imu_mti, const y_imu_t *bias_mti,
    const y_imu_t *imu_altimu, const y_imu_t *bias_altimu, const float *encoder
);

// define these here so we dont have to put the interesting part of the algo under 200 lines of
// matrix math
void ekf_matrix_predict(
    arm_matrix_instance_f32 *P, const arm_matrix_instance_f32 *F, const arm_matrix_instance_f32 *Q
);
void ekf_matrix_correct(
    arm_matrix_instance_f32 *P, arm_matrix_instance_f32 *K, const arm_matrix_instance_f32 *H,
    const arm_matrix_instance_f32 *R, const uint16_t size_measurement
);

#endif