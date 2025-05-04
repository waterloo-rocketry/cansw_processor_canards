#ifndef CONTROLLER_ALGORITHM_H_
#define CONTROLLER_ALGORITHM_H_

#include "application/controller/gain_table.h"
#include "arm_math.h"
#include "third_party/rocketlib/include/common.h"
#include <math.h>

#define FEEDBACK_GAIN_NUM (GAIN_NUM - 1)
#define NEW_ROLL_STATE_NUM FEEDBACK_GAIN_NUM - 1
#define MIN_COOR_BOUND 0

// output related const
extern const double max_commanded_angle; // 10 degrees in radians
extern double reference_signal; // no roll program for test flight
extern const double commanded_angle_zero; // safe mode, init overwrite, p and c out of bound

typedef union {
    double gain_arr[NEW_GAIN_NUM];

    struct {
        double gain_k[NEW_ROLL_STATE_NUM];
        double gain_k_pre;
    };

} controller_gain_t;

w_status_t interpolate_gain(double p_dyn, double coeff, controller_gain_t *gain_output);
w_status_t get_commanded_angle(
    controller_gain_t control_gain, double control_roll_state[NEW_ROLL_STATE_NUM], double *cmd_angle
);

#endif // CONTROLLER_ALGORITHM_H_