#ifndef MOVELLA_H
#define MOVELLA_H

#include <stdint.h>

#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"

typedef struct {
    vector3d_t acc; // (x, y, z) m/s^2
    vector3d_t gyr; // (x, y, z) rad/s
    vector3d_t euler; // (x, y, z) deg
    vector3d_t mag; // (x, y, z) arbitrary units, note: need to be converted to uT later
    float pres; // Pa
    float temp; // °c
} movella_data_t;

// Initialize the xsens interface, pass the configuration to the sensor
w_status_t movella_init(void);

// Return a copy structure of the latest received movella_data_t
w_status_t movella_get_data(movella_data_t *out_data);

#endif
