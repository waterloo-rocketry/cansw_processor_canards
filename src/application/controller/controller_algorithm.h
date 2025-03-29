#ifndef CONTROLLER_ALGORITHM_H_
#define CONTROLLER_ALGORITHM_H_

#include "application/controller/gain_table.h"
#include "arm_math.h"
#include "third_party/rocketlib/include/common.h"

#define FEEDBACK_GAIN_NUM (GAIN_NUM - 1)
#define MIN_COOR_BOUND 0

typedef union {
    float gain_arr[GAIN_NUM];

    struct {
        float gain_k[FEEDBACK_GAIN_NUM];
        float gain_k_pre;
    };

} controller_gain_t;

w_status_t interpolate_gain(float p_dyn, float coeff, controller_gain_t *gain_output);

#endif // CONTROLLER_ALGORITHM_H_