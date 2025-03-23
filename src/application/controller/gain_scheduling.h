#ifndef GAIN_SCHEDULING_H_
#define GAIN_SCHEDULING_H_

#include <math.h>

#define GAIN_NUM 4
#define FEEDBACK_GAIN_NUM (GAIN_NUM - 1)

// for building each interpolation instance
#define GAIN_P_SIZE 200
#define GAIN_C_SIZE 30

typedef union {
    float gain_arr[GAIN_NUM];

    struct {
        float gain_k[FEEDBACK_GAIN_NUM];
        float gain_k_pre;
    };

} controller_gain_t;

static const float max_commanded_angle = 20 * 180.0 / M_PI;

float interpolate_gain(float *p_dyn, float *coeff, int gain_index);

#endif // GAIN_SCHEDULING_H_