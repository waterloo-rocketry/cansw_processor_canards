#ifndef MODEL_IMU_H
#define MODEL_IMU_H

#include "application/estimator/estimator_types.h"
#include "common/math/math.h"
typedef union {
    float array[13];
    struct {
        quaternion_t attitude;
        vector3d_t rates;
        vector3d_t velocity;
        float altitude;
        float CL;
        float delta;
    };
} est_state_t;

// sensor data from 1 arbitrary imu
typedef union {
    float array[10];
    struct {
        vector3d_t accelerometer;
        vector3d_t gyroscope;
        vector3d_t magnetometer;
        float barometer;
    };
} est_imu_data_t;

y_imu_t model_measurement_imu_(x_state_t *est_state, y_imu_t *imu_bias);

#endif