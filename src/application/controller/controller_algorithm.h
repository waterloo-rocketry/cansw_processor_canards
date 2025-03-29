#ifndef CONTROLLER_ALGORITHM_H_
#define CONTROLLER_ALGORITHM_H_

#include "application/controller/gain_table.h"
#include "arm_math.h"
#include "third_party/rocketlib/include/common.h"
#include <math.h>

#define FEEDBACK_GAIN_NUM (GAIN_NUM - 1)
#define MIN_COOR_BOUND 0

// output related const
static const float max_commanded_angle = 20 / 180.0 * M_PI; // 20 degrees in radians
static const float reference_signal = 0.0f; // no roll program for test flight

static const float commanded_angle_zero = 0.0f; // safe mode, init overwrite, p and c out of bound

typedef union {
    float gain_arr[GAIN_NUM];

    struct {
        float gain_k[FEEDBACK_GAIN_NUM];
        float gain_k_pre;
    };

} controller_gain_t;

w_status_t interpolate_gain(float p_dyn, float coeff, controller_gain_t *gain_output);
w_status_t get_commanded_angle(
    controller_gain_t control_gain, float control_roll_state[FEEDBACK_GAIN_NUM], float *cmd_angle
);

#endif // CONTROLLER_ALGORITHM_H_