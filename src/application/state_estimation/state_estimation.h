#ifndef STATE_EST_H
#define STATE_EST_H

#include "controller.h"
#include "can_handler.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /* data */
    float commanded_angle;
    uint32_t timestamp;
} Estimator_control_input_t;

typedef struct
{
    /* data */
    float timestamp_imu;
    vector3d_t accelerometer;
    vector3d_t gyroscope;
    vector3d_t magnometer;
    float barometer;
} Estimator_IMU_input_data;

typedef struct
{
    /* data */
    vector3d_t attitude;
    vector3d_t rates;
    vector3d_t velocity;
    float altitude;
    uint32_t timestamp_con;

} Estimator_controller_state_t;

#define can_output_rate 3;
#define can_output_counter 0;

w_status_t StateEst_Update_Inputs_imu(Estimator_IMU_input_data *data);

void StateEst_task(void *argument);

w_status_t StateEst_Init(Estimator_controller_state_t *data);

#endif