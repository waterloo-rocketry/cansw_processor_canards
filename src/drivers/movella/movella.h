#ifndef MOVELLA_H
#define MOVELLA_H

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "common/math/math.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include "task.h"
#include "third_party/rocketlib/include/common.h"
#include "drivers/uart/uart.h"

typedef struct {
    vector3d_t acc; // (x, y, z) m/s^2
    vector3d_t gyr; // (x, y, z) rad/s
    vector3d_t euler; // (x, y, z) deg
    vector3d_t mag; // (x, y, z) arbitrary units, note: need to be converted to uT later
    float pres; // Pa
    float temp; // °c
} MovellaData_t;

//Initialize the xsens interface and the UART module, pass the configuration to the sensor
w_status_t movella_init(void);

// Resets the orientation
void movella_reset_orientation(void);

// Return a copy structure of the MovellaData_t
w_status_t movella_get_data(MovellaData_t *out_data);

#endif
