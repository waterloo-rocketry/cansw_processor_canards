#ifndef MODEL_IMU_H
#define MODEL_IMU_H

#include "application/estimator/estimator_types.h"
#include "common/math/math.h"


/**
 * @brief Computes measurement prediction using current state and sensor biases
 * @param x_state_t pointer to estimator state
 * @param y_imu_t pointer to sensor biases
 * @return y_imu_t measurement prediction
 */
y_imu_t model_measurement_imu(const x_state_t *est_state, const y_imu_t *imu_bias);


#endif
