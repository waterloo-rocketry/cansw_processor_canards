#ifndef MODEL_IMU_H
#define MODEL_IMU_H

#include "application/estimator/estimator_types.h"
#include "common/math/math.h"
#include "arm_math.h"

/**
 * @brief Computes measurement prediction using current state and sensor biases
 * @param x_state_t pointer to estimator state
 * @param y_imu_t pointer to sensor biases
 * @return y_imu_t measurement prediction
 */
y_imu_t model_measurement_imu(const x_state_t *state, const y_imu_t *imu_bias);

/**
 * @brief jacobian of the measurement model
 * @param 7x13 jacobian matrix pointer to write to
 * @param x_state_t pointer to estimator state
 * @param y_imu_t pointer to sensor biases
 * @param double time step
 */
void model_measurement_imu_jacobian(arm_matrix_instance_f64 *imu_jacobian, const x_state_t *state, const y_imu_t *imu_bias, double dt);

#endif
