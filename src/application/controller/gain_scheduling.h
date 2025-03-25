#ifndef GAIN_SCHEDULING_H_
#define GAIN_SCHEDULING_H_

#include "arm_math.h"
#include <math.h>

#define GAIN_NUM 4
#define FEEDBACK_GAIN_NUM (GAIN_NUM - 1)

typedef union {
    float gain_arr[GAIN_NUM];

    struct {
        float gain_k[FEEDBACK_GAIN_NUM];
        float gain_k_pre;
    };

} controller_gain_t;

extern arm_bilinear_interp_instance_f32 gain_instance;
static const float max_commanded_angle = 20 * 180.0 / M_PI;

// placeholder: suppose this is imported daa
// for normalizing inputs
extern float canard_coeff_scale;
extern float canard_coeff_offset;
extern float pressure_dynamic_scale;
extern float pressure_dynamic_offset;
// for building each interpolation instance
extern const uint16_t numRow;
extern const uint16_t numCol;
static float gain_table[GAIN_NUM][6000] __attribute__((unused));

#endif // GAIN_SCHEDULING_H_