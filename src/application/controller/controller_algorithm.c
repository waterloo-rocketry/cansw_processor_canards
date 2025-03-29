#include "application/controller/controller_algorithm.h"

// gain instances
static arm_bilinear_interp_instance_f32 gain_instance_arr[GAIN_NUM] = {
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[0][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[1][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[2][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[3][0]},
};

w_status_t interpolate_gain(float p_dyn, float coeff, controller_gain_t *gain_output) {
    // Normalize coordinates
    float p_norm = (p_dyn - pressure_dynamic_offset) / pressure_dynamic_scale - 1;
    float c_norm = (coeff - canard_coeff_offset) / canard_coeff_scale - 1;

    // check bounds for p and c
    if ((MIN_COOR_BOUND > p_norm) || (GAIN_P_SIZE - 1 < p_norm) || (MIN_COOR_BOUND > c_norm) ||
        (GAIN_C_SIZE - 1 < c_norm)) {
        return W_FAILURE;
    }

    // Interpolate
    for (int i = 0; i < GAIN_NUM; i++) {
        gain_output->gain_arr[i] = arm_bilinear_interp_f32(&gain_instance_arr[i], p_norm, c_norm);
    }

    return W_SUCCESS;
}