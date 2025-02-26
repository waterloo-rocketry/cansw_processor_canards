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
    // Zero the data
    out_data->acc.component.x = 0.0f;
    out_data->acc.component.y = 0.0f;
    out_data->acc.component.z = 0.0f;

    out_data->gyr.component.x = 0.0f;
    out_data->gyr.component.y = 0.0f;
    out_data->gyr.component.z = 0.0f;

    out_data->euler.component.x = 0.0f;
    out_data->euler.component.y = 0.0f;
    out_data->euler.component.z = 0.0f;

    out_data->mag.component.x = 0.0f;
    out_data->mag.component.y = 0.0f;
    out_data->mag.component.z = 0.0f;

    out_data->pres = 0.0f;
    out_data->temp = 0.0f;

    return W_SUCCESS;
}
