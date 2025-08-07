#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include "task.h"

#include "third_party/xsens-mti/src/xsens_mti.h"

#include "common/math/math.h"
#include "drivers/movella/movella.h"
#include "drivers/uart/uart.h"

#define UART_TX_TIMEOUT_MS 100
// should be every 5 ms but allow some leeway before erroring
#define UART_RX_TIMEOUT_MS 10
#define XSENS_ARR_ELEM 7

typedef struct {
    xsens_interface_t xsens_interface;
    SemaphoreHandle_t data_mutex;
    TaskHandle_t task_handle;
    movella_data_t latest_data;
    bool initialized;
    bool configured;
} movella_state_t;

static movella_state_t s_movella = {0};

static void movella_event_callback(XsensEventFlag_t event, XsensEventData_t *mtdata) {
    if (xSemaphoreTake(s_movella.data_mutex, 0) == pdTRUE) {
        switch (event) {
            case XSENS_EVT_ACCELERATION:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
                    s_movella.latest_data.acc.x = mtdata->data.f4x3[0];
                    s_movella.latest_data.acc.y = mtdata->data.f4x3[1];
                    s_movella.latest_data.acc.z = mtdata->data.f4x3[2];
                }
                break;

            case XSENS_EVT_RATE_OF_TURN:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
                    s_movella.latest_data.gyr.x = mtdata->data.f4x3[0];
                    s_movella.latest_data.gyr.y = mtdata->data.f4x3[1];
                    s_movella.latest_data.gyr.z = mtdata->data.f4x3[2];
                }
                break;

            case XSENS_EVT_MAGNETIC:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
                    s_movella.latest_data.mag.x = mtdata->data.f4x3[0];
                    s_movella.latest_data.mag.y = mtdata->data.f4x3[1];
                    s_movella.latest_data.mag.z = mtdata->data.f4x3[2];
                }
                break;

            case XSENS_EVT_QUATERNION:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT4) {
                    float euler_rad[3] = {0};
                    // xsens_quaternion_to_euler(mtdata->data.f4x4, euler_rad);
                    // TODO: add quaternion function once implemented

                    s_movella.latest_data.euler.x = euler_rad[0] * DEG_PER_RAD;
                    s_movella.latest_data.euler.y = euler_rad[1] * DEG_PER_RAD;
                    s_movella.latest_data.euler.z = euler_rad[2] * DEG_PER_RAD;
                }
                break;

            case XSENS_EVT_PRESSURE:
                if (mtdata->type == XSENS_EVT_TYPE_U32) {
                    s_movella.latest_data.pres = (float)mtdata->data.u4;
                }
                break;

            case XSENS_EVT_TEMPERATURE:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT) {
                    s_movella.latest_data.temp = mtdata->data.f4;
                }
                break;
            default:
                // Need a default case to avoid compiler warning (error)
                break;
        }

        xSemaphoreGive(s_movella.data_mutex);
    }
}

static void movella_uart_send(uint8_t *data, uint16_t length) {
    (void)uart_write(UART_MOVELLA, data, length, UART_TX_TIMEOUT_MS);
}

w_status_t movella_init(void) {
    if (s_movella.initialized) {
        return W_SUCCESS;
    }

    s_movella.data_mutex = xSemaphoreCreateMutex();

    if (s_movella.data_mutex == NULL) {
        return W_FAILURE;
    }

    s_movella.xsens_interface.event_cb = movella_event_callback;
    s_movella.xsens_interface.output_cb = movella_uart_send;

    s_movella.initialized = true;
    return W_SUCCESS;
}

w_status_t movella_get_data(movella_data_t *out_data, uint32_t timeout_ms) {
    if (NULL == out_data) {
        return W_INVALID_PARAM;
    }

    if (!s_movella.initialized) {
        return W_FAILURE;
    }

    if (pdTRUE == xSemaphoreTake(s_movella.data_mutex, pdMS_TO_TICKS(timeout_ms))) {
        *out_data = s_movella.latest_data;
        xSemaphoreGive(s_movella.data_mutex);
        return W_SUCCESS;
    }

    return W_FAILURE;
}

// NOTE: removed in favour of using xsens app to config all movellas the same
// static void movella_configure(void) {
//     XsensFrequencyConfig_t settings[XSENS_ARR_ELEM] = {
//         {.id = XDI_QUATERNION, .frequency = 200}, // 5ms
//         {.id = XDI_ACCELERATION, .frequency = 200}, // 5ms
//         {.id = XDI_RATE_OF_TURN, .frequency = 200}, // 5ms
//         {.id = XDI_MAGNETIC_FIELD, .frequency = 100}, // 10ms
//         {.id = XDI_TEMPERATURE, .frequency = 5}, // 200ms
//         {.id = XDI_BARO_PRESSURE, .frequency = 40}, // 25ms
//         {.id = XDI_STATUS_WORD, .frequency = 0xFFFF}
//     };

//     xsens_mti_request(&s_movella.xsens_interface, MT_GOTOCONFIG);
//     vTaskDelay(pdMS_TO_TICKS(100));

//     xsens_mti_set_configuration(&s_movella.xsens_interface, settings, XSENS_ARR_ELEM);
//     vTaskDelay(pdMS_TO_TICKS(100));

//     xsens_mti_request(&s_movella.xsens_interface, MT_GOTOMEASUREMENT);
//     vTaskDelay(pdMS_TO_TICKS(100));

//     s_movella.configured = true;
// }

// store this as static var instead of inside task to avoid using excessive task stack space
static uint8_t movella_rx_buffer[UART_MAX_LEN] = {0};

void movella_task(void *parameters) {
    (void)parameters;
    uint16_t rx_length;

    while (1) {
        w_status_t status =
            uart_read(UART_MOVELLA, movella_rx_buffer, &rx_length, UART_RX_TIMEOUT_MS);

        // TODO: avoid race condition on s_movella.latest_data.is_dead cuz it could be read by
        // imu handler while this is in progress? idt it matters in practice much cuz its a bool and
        // should rarely change values so its fine to keep this sus for now. doesn't affect
        // functionality that we need
        if ((W_SUCCESS == status) && (rx_length > 0) && (rx_length < UART_MAX_LEN)) {
            xsens_mti_parse_buffer(&s_movella.xsens_interface, movella_rx_buffer, rx_length);
            s_movella.latest_data.is_dead = false;
        } else {
            s_movella.latest_data.is_dead = true;
        }
    }
}
