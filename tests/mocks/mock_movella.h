#ifndef MOCK_MOVELLA_H
#define MOCK_MOVELLA_H

#include "fff.h"
#include "third_party/rocketlib/include/common.h"
#include "common/math/math.h"

// Define the Movella data structure for mocking
typedef struct
{
    vector3d_t acc;
    vector3d_t gyr;
    vector3d_t mag;
    float pres;
    float temp;
} movella_data_t;

// Declare FFF fakes for all Movella functions
DECLARE_FAKE_VALUE_FUNC(w_status_t, movella_init);
DECLARE_FAKE_VALUE_FUNC(w_status_t, movella_get_data, movella_data_t *);

#endif // MOCK_MOVELLA_H