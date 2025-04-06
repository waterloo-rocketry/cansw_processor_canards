/**
 * Types used by the Estimator internally
 */
#ifndef EST_TYPE_H
#define EST_TYPE_H

#include "common/math/math.h"

/*
* State
*/ 
typedef union {
    float array[13];
    struct __attribute__((packed)) {
        quaternion_t attitude;
        vector3d_t rates;
        vector3d_t velocity;
        float altitude;
        float CL;
        float delta;
    };
} x_state_t;

/*
* Input signal
*/ 
typedef struct {
    float cmd;
    vector3d_t acceleration;
} u_dynamics_t;

/*
* Measurement signals
*/ 
// sensor data from 1 arbitrary imu
// Pad filter: both filtered_i and bias_i can be of this type
// EKF: bias_i, y_i, and h_x_i are of this type
// IMU measurement model: the return is of this type
typedef union {
    float array[10];
    struct  __attribute__((packed)) {
        vector3d_t accelerometer;
        vector3d_t gyroscope;
        vector3d_t magnetometer;
        float barometer;
    };
} y_imu_t;

#endif