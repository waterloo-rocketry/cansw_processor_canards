#ifndef MODEL_IMU_H
#define MODEL_IMU_H

#include "application/estimator/estimator_types.h"
#include "common/math/math.h"

y_imu_t model_measurement_imu(x_state_t *est_state, y_imu_t *imu_bias);

#endif
