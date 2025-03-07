#include "application/can_handler/can_handler.h"

// stub definition to compile
w_status_t can_register_callback(can_msg_type_t msg_type, can_callback_t callback) {
    (void)msg_type;
    (void)callback;
    return W_SUCCESS;
}
