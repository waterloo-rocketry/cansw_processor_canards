#ifndef CONTROLLER_ALGORITHM_H_
#define CONTROLLER_ALGORITHM_H_

#include "application/controller/gain_table.h"
#include "arm_math.h"
#include "third_party/rocketlib/include/common.h"
#include <math.h>

#define FEEDBACK_GAIN_NUM (GAIN_NUM - 1) // last gain is pre-gain
#define ROLL_STATE_NUM (FEEDBACK_GAIN_NUM)
#define MIN_COOR_BOUND 0

typedef union {
    double gain_arr[GAIN_NUM];

    struct {
        double gain_k[ROLL_STATE_NUM];
        double gain_k_pre;
    };

} controller_gain_t;

w_status_t interpolate_gain(double p_dyn, double canard_coeff, controller_gain_t *gain_output);

w_status_t get_commanded_angle(
    controller_gain_t control_gain, double control_roll_state[ROLL_STATE_NUM], float ref_signal,
    double *cmd_angle
);

#endif // CONTROLLER_ALGORITHM_H_
