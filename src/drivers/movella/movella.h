#ifndef MOVELLA_H
#define MOVELLA_H

#include <stdint.h>

#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"

typedef struct {
    vector3d_t acc; // (x, y, z) m/s^2
    vector3d_t gyr; // (x, y, z) rad/s
    vector3d_t euler; // (x, y, z) deg
    vector3d_t mag; // (x, y, z) "arbitrary units" - estimator doesnt need conversion so leave this
    float pres; // Pa
    float temp; // °c
} movella_data_t;

/**
 * Initialize the xsens interface, communicate the configuration over uart.
 * takes ~1 second to complete.
 * must be called after uart init and after scheduler start
 * @param huart HAL UART handle for movella
 */
w_status_t movella_init(UART_HandleTypeDef *huart);

/**
 * must be called in uart isr when full movella msg is received
 * @param len number of bytes received
 */
void movella_uart_rx_cb(uint32_t len);

// Return a copy structure of the latest received movella_data_t
w_status_t movella_get_data(movella_data_t *out_data, uint32_t timeout_ms);

// FreeRTOS task function
void movella_task(void *parameters);
#endif
