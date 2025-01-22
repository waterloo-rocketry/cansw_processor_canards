#ifndef STATE_EST_H
#define STATE_EST_H

#include "can_handler.h"
#include "common/math/math.h"
#include "controller.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* data */
    float commanded_angle;
    uint32_t timestamp;
} Estimator_control_input_t;

typedef struct {
    /* data */
    float timestamp_imu;
    vector3d_t accelerometer;
    vector3d_t gyroscope;
    vector3d_t magnometer;
    float barometer;
} Estimator_IMU_input_data;

w_status_t Estimator_Update_Inputs_imu(Estimator_IMU_input_data *data);

void Estimator_task(void *argument);

w_status_t Estimator_Init(control_output_SE_t *data);

#endif