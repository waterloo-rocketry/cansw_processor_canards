#include "application/controller/controller_algorithm.h"
#include "common/math/math.h"

static const double max_commanded_angle = 10 * RAD_PER_DEG; // 10 degrees in radians

// gain instances
static arm_bilinear_interp_instance_f32 gain_instance_arr[GAIN_NUM] = {
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[0][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[1][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[2][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[3][0]},
};

w_status_t interpolate_gain(double p_dyn, double canard_coeff, controller_gain_t *gain_output) {
    // Normalize coordinates
    double p_norm = (p_dyn - PRESSURE_DYNAMIC_OFFSET) / PRESSURE_DYNAMIC_SCALE;
    double c_norm = (canard_coeff - CANARD_COEFF_OFFSET) / CANARD_COEFF_SCALE;

    // check bounds for p and c: for debugging
    if ((MIN_COOR_BOUND > p_norm) || (GAIN_P_SIZE - 1 < p_norm) || (MIN_COOR_BOUND > c_norm) ||
        (GAIN_C_SIZE - 1 < c_norm)) {
        return W_FAILURE;
    }

    // Interpolate
    for (int i = 0; i < GAIN_NUM; i++) {
        gain_output->gain_arr[i] = arm_bilinear_interp_f32(&gain_instance_arr[i], c_norm, p_norm);
    }

    return W_SUCCESS;
}

w_status_t get_commanded_angle(
    controller_gain_t control_gain, double control_roll_state[ROLL_STATE_NUM], float ref_signal,
    double *cmd_angle
) {
    // compute commanded angle
    double dot_prod = 0.0;
    double output = 0.0;
    for (int i = 0; i < ROLL_STATE_NUM; i++) {
        dot_prod += control_gain.gain_k[i] * control_roll_state[i];
    }

    output = dot_prod + control_gain.gain_k_pre * ref_signal;

    output = fmin(fmax(output, -max_commanded_angle), max_commanded_angle);

    *cmd_angle = output;
    return W_SUCCESS;
}