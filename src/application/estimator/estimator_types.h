/**
 * Types used by the Estimator internally
 */
#ifndef EST_TYPE_H
#define EST_TYPE_H

#include "common/math/math.h"

// size of the y_imu_t array
#define Y_IMU_SIZE_ITEMS 10
// size of the x_state_t array
#define X_STATE_SIZE_ITEMS 13

/*
 * State
 */
typedef union {
    double array[X_STATE_SIZE_ITEMS];
    struct {
        quaternion_t attitude;
        vector3d_t rates;
        vector3d_t velocity;
        double altitude;
        double CL;
        double delta;
    };
} x_state_t;

/*
 * Input signal
 */
typedef struct {
    double cmd;
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
    double array[Y_IMU_SIZE_ITEMS];
    struct {
        vector3d_t accelerometer;
        vector3d_t gyroscope; 
        vector3d_t magnetometer; 
        double barometer;
    };
} y_imu_t;

#endif