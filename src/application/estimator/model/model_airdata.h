/**
* Atmospheric model
* uses US standard atmosphere, after: Stengel 2004 - Flight Dynamics
*/
#ifndef MODEL_AIRDATA_H
#define MODEL_AIRDATA_H

#include <math.h>

typedef struct
{
    float pressure;
    float temperature;
    float density;
    float mach_local;
} estimator_airdata_t;

// altdata function uses barometric pressure to determine current altitude (inside Troposphere)
float model_altdata(float pressure);

// airdata function uses altitude to return pressure, temperature, density, local mach
estimator_airdata_t model_airdata(float altitude);

//   void model_airdata_biasfinder(void)

#endif