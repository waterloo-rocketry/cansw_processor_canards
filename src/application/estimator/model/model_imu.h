#ifndef MODEL_IMU_H
#define MODEL_IMU_H

#include "common/math/math.h"

typedef union {
    float array[13];
    struct {
        quaternion_t attitude;
        vector3d_t rates;
        vector3d_t velocity;
        float altitude;
        float CL;
        float delta;
    }
} estimator_state_t;






#endif 