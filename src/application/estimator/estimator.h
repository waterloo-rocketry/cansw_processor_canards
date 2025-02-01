#ifndef STATE_EST_H
#define STATE_EST_H

#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    /* data */
    float commanded_angle;
    uint32_t timestamp;
} estimator_controller_input_t;

typedef struct {
    /* data */
    float timestamp_imu;
    vector3d_t accelerometer;
    vector3d_t gyroscope;
    vector3d_t magnometer;
    float barometer;
} estimator_imu_input_data;

w_status_t estimator_update_inputs_imu(estimator_imu_input_data *data);

void estimator_task(void);

w_status_t estimator_init(estimator_controller_input_t *data);

#endif