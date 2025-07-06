#include "application/controller/controller_module.h"
#include "application/controller/controller_algorithm.h"
#include "application/logger/log.h"
#include "common/math/math.h"

w_status_t controller_module(
    controller_input_t input, uint32_t flight_ms, double *output_angle, float *ref_signal
) {
    // ref_signal
    float r = 0.0f;
    controller_gain_t controller_gain = {0};

    // validate inputs
    if (output_angle == NULL || ref_signal == NULL) {
        log_text(10, "cntlmodule", "nullptrs");
        return W_INVALID_PARAM;
    }

    // %% Reference signal
    // % Generates reference signal for roll program
    // % includes multiple roll angle steps. Reference r [rad].
    if (flight_ms > 10 * MS_PER_SEC) {
        if (flight_ms < 15 * MS_PER_SEC) {
            r = 0.5;
        } else if (flight_ms < 22 * MS_PER_SEC) {
            r = -0.5;
        } else if (flight_ms < 28 * MS_PER_SEC) {
            r = 0.5;
        } else if (flight_ms > 36 * MS_PER_SEC) {
            r = 0;
        }
    } else {
        log_text(10, "cntlmodule", "invalid flight ms");
        return W_FAILURE;
    }

    // %% controller algorithm
    // % Computes control output. Uses gain schedule table and simplified roll model
    // % Inputs: roll state input(1:3), flight conditions input(4:5), reference signal r
    // % Outputs: control input u

    // %%% Gain scheduling
    if (interpolate_gain(input.pressure_dynamic, input.canard_coeff, &controller_gain) !=
        W_SUCCESS) {
        log_text(10, "cntlmodule", "gain interp fail");
        return W_FAILURE;
    }

    // %%% Feedback law
    if (get_commanded_angle(controller_gain, input.roll_state.roll_state_arr, r, output_angle) !=
        W_SUCCESS) {
        log_text(10, "cntlmodule", "get cmd fail");
        return W_FAILURE;
    }

    *ref_signal = r;

    return W_SUCCESS;
}
