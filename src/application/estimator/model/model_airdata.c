
#include "application/estimator/model/model_airdata.h"

#include "application/estimator/estimator.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <stdbool.h>
#include <stdint.h>

// physical constants
#define AIR_GAMMA 1.4      // adiabatic index for air
#define AIR_R 287.0579     // specific gas constant
#define EARTH_R0 6356766.0 // mean earth radius
#define EARTH_G0 9.8       // gravitational acceleration

typedef struct
{
    double base_height; // altitude, for which following constants are defined for
    double base_pressure;
    double base_temperature;
    double base_lapse_rate; // temperature change with altitude
} atmosphere_layer_t;       // struct for one layer of atmoshphere

static const atmosphere_layer_t air_atmosphere[] = {
    {0, 101325, 288.15, 0.0065},     // Troposphere
    {11000, 22632.1, 216.65, 0},     // Tropopause
    {20000, 5474.9, 216.65, -0.001}, // Stratosphere
    {32000, 868.02, 228.65, -0.0028} // Stratosphere 2
};

// airdata function uses altitude to return pressure, temperature, density, local mach
estimator_model_airdata_t model_airdata(float altitude)
{
    estimator_model_airdata_t result;

    // Altitude to geopotential altitude
    altitude = EARTH_R0 * altitude / (EARTH_R0 - altitude);

    // Select atmosphere layer
    const atmosphere_layer_t *layer = &air_atmosphere[0]; // start with Troposphere
    if (altitude > air_atmosphere[1].base_height)         // check if altitude is in higher atmosphere layers
    {
        if (altitude < air_atmosphere[2].base_height)
        {
            layer = &air_atmosphere[1];
        }
        else if (altitude < air_atmosphere[3].base_height)
        {
            layer = &air_atmosphere[2];
        }
        else
        {
            layer = &air_atmosphere[3];
        }
    }
    double b = layer->base_height;
    double P_B = layer->base_pressure;
    double T_B = layer->base_temperature;
    double k = layer->base_lapse_rate;

    // temperature
    result.temperature = T_B - k * (altitude - b);

    // static pressure, different formulas for lapse rate / no lapse rate
    if (k == 0)
    {
        result.pressure = P_B * exp(-EARTH_G0 * (altitude - b) / (AIR_R * T_B));
    }
    else
    {
        result.pressure = P_B * pow(1 - (k / T_B) * (altitude - b), EARTH_G0 / (AIR_R * k));
    }

    // density
    result.density = result.pressure / (AIR_R * result.temperature);

    // local speed of sound
    result.mach_local = sqrt(AIR_GAMMA * AIR_R * result.temperature);

    return result;
}