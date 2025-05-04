#include "application/controller/controller_algorithm.h"

const float max_commanded_angle = 10 / 180.0 * M_PI; // 10 degrees in radians
float reference_signal = 0.0f; // no roll program for test flight

const float commanded_angle_zero = 0.0f; // safe mode, init overwrite, p and c out of bound

// gain instances
static arm_bilinear_interp_instance_f32 gain_instance_arr[NEW_GAIN_NUM] = {
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[0][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[1][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[2][0]}
    // ,
    // {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[3][0]},
};

w_status_t interpolate_gain(float p_dyn, float coeff, controller_gain_t *gain_output) {
    // **new 
    // temporary overwrite for canard coeff
    coeff = 0.5f;
    
    // Normalize coordinates
    float p_norm = (p_dyn - PRESSURE_DYNAMIC_OFFSET) / PRESSURE_DYNAMIC_SCALE;
    float c_norm = (coeff - CANARD_COEFF_OFFSET) / CANARD_COEFF_SCALE;

    // check bounds for p and c: for debugging
    if ((MIN_COOR_BOUND > p_norm) || (GAIN_P_SIZE - 1 < p_norm) || (MIN_COOR_BOUND > c_norm) ||
        (GAIN_C_SIZE - 1 < c_norm)) {
        return W_FAILURE;
    }

    // Interpolate
    for (int i = 0; i < NEW_GAIN_NUM; i++) {
        gain_output->gain_arr[i] = arm_bilinear_interp_f32(&gain_instance_arr[i], c_norm, p_norm);
    }

    return W_SUCCESS;
}

w_status_t get_commanded_angle(
    controller_gain_t control_gain, float control_roll_state[NEW_ROLL_STATE_NUM], float *cmd_angle
) {
    // compute commanded angle
    float dot_prod = 0.0f;
    float output = 0.0f;
    arm_dot_prod_f32(control_gain.gain_k, control_roll_state, NEW_ROLL_STATE_NUM, &dot_prod);
    output = dot_prod + control_gain.gain_k_pre * reference_signal;

    output = fmin(fmax(output, -max_commanded_angle), max_commanded_angle);

    *cmd_angle = output;
    return W_SUCCESS;
}