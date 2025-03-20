#ifndef GAIN_SCHEDULING_H_
#define GAIN_SCHEDULING_H_

#include "arm_math.h"
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

extern arm_bilinear_interp_instance_f32 gain_instance;
static const float max_commanded_angle = 20 * 180.0 / M_PI;

extern const float gain_table[GAIN_NUM][GAIN_P_SIZE * GAIN_C_SIZE];

// Gain table information: for normalizing inputs
extern const float pressure_dynamic_scale;
extern const float canard_coeff_scale;

extern const float pressure_dynamic_offset;
extern const float canard_coeff_offset;

#endif // GAIN_SCHEDULING_H_