#include "Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h"
#include "Middlewares/Third_Party/FreeRTOS/Source/include/task.h"
#include "Middlewares/Third_Party/FreeRTOS/Source/include/queue.h"
#include "Middlewares/Third_Party/FreeRTOS/Source/include/semphr.h"
#include "stm32h7xx_hal.h"

#include "third_party/xsens-mti/src/xsens_mti.h"

#include "drivers/movella/movella.h"
#include "drivers/uart/uart.h"

#define MOVELLA_OUTPUT_RATE 100
#define UART_TIMEOUT 100

#define MOVELLA_STACK_SIZE     (configMINIMAL_STACK_SIZE * 3)
#define MOVELLA_TASK_PRIORITY  (tskIDLE_PRIORITY + 2)

typedef struct {
    xsens_interface_t xsens_interface;
    SemaphoreHandle_t data_mutex;
    TaskHandle_t task_handle;
    movella_data_t latest_data;
    bool initialized;
} movella_state_t;

static movella_state_t s_movella = {0};

static void movella_task(void *parameters);
static void movella_event_callback(XsensEventFlag_t event, XsensEventData_t *mtdata);
static void movella_uart_send(uint8_t *data, uint16_t length);

w_status_t movella_init(void) {
    if (s_movella.initialized) {
        return W_SUCCESS;
    }

    s_movella.data_mutex = xSemaphoreCreateMutex();

    if (s_movella.data_mutex == NULL) {
        return W_FAILURE;
    }

    s_movella.xsens_interface.event_cb = &movella_event_callback;
    s_movella.xsens_interface.output_cb = &movella_uart_send;

    BaseType_t task_created = xTaskCreate(
        movella_task,
        "Movella",
        MOVELLA_STACK_SIZE,
        NULL,
        MOVELLA_TASK_PRIORITY,
        &s_movella.task_handle
    );

    if (task_created != pdPASS) {
        vSemaphoreDelete(s_movella.data_mutex);
        return W_FAILURE;
    }
    
    return W_SUCCESS;
}

w_status_t movella_get_data(movella_data_t *out_data) {
    if (!s_movella.initialized || out_data == NULL) {
        return W_INVALID_PARAM;
    }
    
    if (xSemaphoreTake(s_movella.data_mutex, portMAX_DELAY) == pdTRUE) {
        *out_data = s_movella.latest_data;
        xSemaphoreGive(s_movella.data_mutex);
        return W_SUCCESS;
    }
    
    return W_FAILURE;
}

static void movella_event_callback(XsensEventFlag_t event, XsensEventData_t *mtdata) {
    if (xSemaphoreTake(s_movella.data_mutex, 0) == pdTRUE) {
        switch (event) {
            case XSENS_EVT_ACCELERATION:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
                    s_movella.latest_data.acc.component.x = mtdata->data.f4x3[0];
                    s_movella.latest_data.acc.component.y = mtdata->data.f4x3[1];
                    s_movella.latest_data.acc.component.z = mtdata->data.f4x3[2];
                }
                break;
                
            case XSENS_EVT_RATE_OF_TURN:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
                    s_movella.latest_data.gyr.component.x = mtdata->data.f4x3[0];
                    s_movella.latest_data.gyr.component.y = mtdata->data.f4x3[1];
                    s_movella.latest_data.gyr.component.z = mtdata->data.f4x3[2];
                }
                break;
                
            case XSENS_EVT_MAGNETIC:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
                    s_movella.latest_data.mag.component.x = mtdata->data.f4x3[0];
                    s_movella.latest_data.mag.component.y = mtdata->data.f4x3[1];
                    s_movella.latest_data.mag.component.z = mtdata->data.f4x3[2];
                }
                break;
                
            case XSENS_EVT_QUATERNION:
                if (mtdata->type == XSENS_EVT_TYPE_FLOAT4) {
                    float euler_rad[3];
                    xsens_quaternion_to_euler(mtdata->data.f4x4, euler_rad);
                    
                    s_movella.latest_data.euler.component.x = euler_rad[0] * (180.0f / 3.14159f);
                    s_movella.latest_data.euler.component.y = euler_rad[1] * (180.0f / 3.14159f);
                    s_movella.latest_data.euler.component.z = euler_rad[2] * (180.0f / 3.14159f);
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
        }
        
        xSemaphoreGive(s_movella.data_mutex);
    }
}

static void movella_configure(void) {
    XsensFrequencyConfig_t settings[] = {
        { .id = XDI_QUATERNION, .frequency = 100 },
        { .id = XDI_ACCELERATION, .frequency = 100 },
        { .id = XDI_RATE_OF_TURN, .frequency = 100 },
        { .id = XDI_MAGNETIC_FIELD, .frequency = 50 },
        { .id = XDI_TEMPERATURE, .frequency = 5 },
        { .id = XDI_BARO_PRESSURE, .frequency = 5 },
        { .id = XDI_STATUS_WORD, .frequency = 0xFFFF }
    };
    
    xsens_mti_request(&s_movella.xsens_interface, MT_GOTOCONFIG);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    xsens_mti_set_configuration(&s_movella.xsens_interface, settings, XSENS_ARR_ELEM(settings));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    xsens_mti_request(&s_movella.xsens_interface, MT_GOTOMEASUREMENT);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void movella_task(void *parameters) {
    uint8_t rx_buffer[UART_MAX_LEN];
    uint16_t rx_length;
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    movella_configure();
    
    s_movella.initialized = true;
    
    while (1) {
        w_status_t status = uart_read(UART_MOVELLA, rx_buffer, &rx_length, 10);
        
        if (status == W_SUCCESS && rx_length > 0) {
            xsens_mti_parse_buffer(&s_movella.xsens_interface, rx_buffer, rx_length);
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}