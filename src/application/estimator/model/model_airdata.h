#ifndef MODEL_AIRDATA_H
#define MODEL_AIRDATA_H

#include "application/estimator/estimator.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /* data */
    float pressure;
    float temperature;
    float density;
    float mach_local;
} estimator_model_airdata_t;

estimator_model_airdata_t model_airdata(float altitude);

//   void model_airdata_biasfinder(void)

#endif