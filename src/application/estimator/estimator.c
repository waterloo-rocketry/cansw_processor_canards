#include "application/estimator/estimator.h"

// Stub implementation
w_status_t estimator_init() {
    return W_SUCCESS;
}

w_status_t estimator_update_inputs_imu(estimator_all_imus_input_t *data) {
    (void)data; // Explicitly mark parameter as unused
    // Just a stub that accepts data but does nothing with it
    return W_SUCCESS;
}
