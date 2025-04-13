#include "application/controller/controller_algorithm.h"

const float max_commanded_angle = 10; // 10 degrees in radians
float reference_signal = 0.0f; // no roll program for test flight

const float commanded_angle_zero = 0.0f; // safe mode, init overwrite, p and c out of bound

// gain instances
static arm_bilinear_interp_instance_f32 gain_instance_arr[GAIN_NUM] __attribute__((unused)) = {
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[0][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[1][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[2][0]},
    {.numRows = GAIN_P_SIZE, .numCols = GAIN_C_SIZE, .pData = &gain_table[3][0]},
};

// testing
static int cycle_counter = 0;

w_status_t interpolate_gain(float p_dyn, float coeff, controller_gain_t *gain_output) {
    return W_SUCCESS;
}

w_status_t get_commanded_angle(
    controller_gain_t control_gain, float control_roll_state[FEEDBACK_GAIN_NUM], float *cmd_angle
) {
    if (cycle_counter > NUM_CMD) {
        cycle_counter = NUM_CMD - 1;
    } else {
        cycle_counter++;
    }
    *cmd_angle = controller_cmd[cycle_counter];
    return W_SUCCESS;
}