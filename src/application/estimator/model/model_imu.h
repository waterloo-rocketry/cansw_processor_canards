#ifndef MODEL_IMU_H
#define MODEL_IMU_H

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

est_imu_data_t model_measurement_imu_mti630(est_state_t *est_state, est_imu_data_t *imu_bias);
est_imu_data_t model_measurement_imu_altimu(est_state_t *est_state, est_imu_data_t *imu_bias);


#endif 