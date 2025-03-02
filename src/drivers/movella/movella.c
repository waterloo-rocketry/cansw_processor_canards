#include "FreeRTOS.h"
#include "semphr.h"

#include "stm32h7xx_hal.h"
#include "third_party/xsens-mti/src/xsens_mti.h"

#include "drivers/movella/movella.h"
#include "drivers/uart/uart.h"

w_status_t movella_init(void) {
    return W_SUCCESS;
}

w_status_t movella_reset_orientation(void) {
    return W_SUCCESS;
}

w_status_t movella_get_data(movella_data_t *out_data) {
    return W_SUCCESS;
}
