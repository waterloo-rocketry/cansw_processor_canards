#ifndef STATE_EST_H
#define STATE_EST_H

#include "application/controller/controller.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

// sensor data from 1 arbitrary imu
typedef union {
    float array[10];
    struct {
        vector3d_t accelerometer;
        vector3d_t gyroscope;
        vector3d_t magnetometer;
        float barometer;
    };
} imu_data_t;

// measurement from 1 arbitrary imu
typedef struct {
    imu_data_t sensor;
    uint32_t timestamp;
    bool is_dead;
} estimator_measurement_imu_t;

// measurement from encoder
typedef struct {
    float sensor;
    uint32_t timestamp;
    bool is_dead;
} estimator_measurement_encoder_t;

// measurement from Movella MTUs internal filter
typedef struct {
    quaternion_t sensor;
    uint32_t timestamp;
    bool is_dead;
} estimator_measurement_mtuAHRS_t;


/**
 * @brief Used to update the imu inputs for estimator
 *
 * @note Should be called every 5ms or faster for optimal estimator performance
 *
 * @param data Pointer to the struct containing latest measurements from all imus
 */
w_status_t estimator_update_inputs_imu(estimator_imu_measurement_t polulu, estimator_imu_measurement_t movella);

void estimator_task(void *argument);

#endif