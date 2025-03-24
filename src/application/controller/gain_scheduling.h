#ifndef GAIN_SCHEDULING_H_
#define GAIN_SCHEDULING_H_

#include "arm_math.h"

#define GAIN_NUM 4
#define FEEDBACK_GAIN_NUM (GAIN_NUM - 1)

typedef union {
    float gain_arr[GAIN_NUM];

    struct {
        float gain_k[FEEDBACK_GAIN_NUM];
        float gain_k_pre;
    };

} controller_gain_t;

float interpolate_gain(float p_dyn, float coeff, controller_gain_t *gain_output);

#endif // GAIN_SCHEDULING_H_