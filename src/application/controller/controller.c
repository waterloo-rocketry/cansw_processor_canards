#include "application/controller/controller.h"

w_status_t controller_get_latest_output(controller_output_t *output) {
    if (output == NULL) {
        return W_FAILURE;
    }
    return W_SUCCESS;
}

w_status_t controller_update_inputs(controller_input_t *new_state) {
    if (new_state == NULL) {
        return W_FAILURE;
    }
    return W_SUCCESS;
}