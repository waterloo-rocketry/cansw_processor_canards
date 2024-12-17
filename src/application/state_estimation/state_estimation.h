#ifdef STATE_EST_H_
#define STATE_EST_H_


#include "controller.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /* data */
    float commanded_angle;
    uint32_t timestamp;
} control_output_SE_t;

typedef struct 
{
    /* data */
    float x;
    float y;
    float z;
} vectors_3axis;


typedef struct 
{
    /* data */
   float timestamp_imu;
    vectors_3axis accelerometer;
    vectors_3axis gyroscope;
    vectors_3axis magnometer;
    float barometer;
} IMU_data;

typedef struct 
{
    /* data */
    vectors_3axis attitude;
    vectors_3axis rates;
    vectors_3axis velocity;
    float altitude;
    uint32_t timestamp_con;
    
} controller_state_t;

#define can_output_rate 3;
#define can_output_counter 0;

w_status_t StateEst_Update_Inputs_imu(IMU_data *data);

void StateEst_task(void *argument);

w_status_t StateEst_Init(controller_state_t *data);

#endif