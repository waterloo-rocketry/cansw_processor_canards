#include "application/estimator/model/model_airdata.h"
#include <math.h>

// physical constants
#define AIR_GAMMA 1.4f // adiabatic index for air
#define AIR_R 287.0579f // specific gas constant
#define EARTH_R0 6356766.0f // mean earth radius
#define EARTH_G0 9.81f // gravitational acceleration

typedef struct {
    float base_height; // altitude, for which following constants are defined for
    float base_pressure;
    float base_temperature;
    float base_temperature_lapse_rate; // temperature change with altitude
} atmosphere_layer_t; // struct for one layer of atmoshphere

static const atmosphere_layer_t air_atmosphere[] = {
    {    0.0, 101325.00, 288.15,  0.0065}, // Troposphere
    {11000.0,  22632.10, 216.65,  0.0000}, // Tropopause
    {20000.0,   5474.90, 216.65, -0.0010}, // Stratosphere
    {32000.0,    868.02, 228.65, -0.0028} // Stratosphere 2
};

// altdata function uses barometric pressure to determine current altitude (inside Troposphere)
float model_altdata(float pressure) {
    float altitude = 0;

    // Select Troposphere
    const float b   = air_atmosphere[0].base_height;
    const float P_B = air_atmosphere[0].base_pressure;
    const float T_B = air_atmosphere[0].base_temperature;
    const float k   = air_atmosphere[0].base_temperature_lapse_rate;

    // inverse barometric formula, for Troposphere
    altitude = b + (T_B / k) * (1.0f - powf(pressure / P_B, (AIR_R * k) / EARTH_G0));

    // Geopotential altitude to normal altitude
    altitude = altitude * EARTH_R0 / (altitude + EARTH_R0);
    return altitude;
}

// airdata function uses altitude to return pressure, temperature, density, local mach
estimator_airdata_t model_airdata(float altitude) {
    estimator_airdata_t result;

    // Altitude to geopotential altitude
    altitude = EARTH_R0 * altitude / (EARTH_R0 - altitude);

    // Select atmosphere layer
    const atmosphere_layer_t *layer = &air_atmosphere[0]; // start with Troposphere
    if (altitude >
        air_atmosphere[1].base_height) // check if altitude is in higher atmosphere layers
    {
        if (altitude < air_atmosphere[2].base_height) {
            layer = &air_atmosphere[1];
        } else if (altitude < air_atmosphere[3].base_height) {
            layer = &air_atmosphere[2];
        } else {
            layer = &air_atmosphere[3];
        }
    }
    const float b   = layer->base_height;
    const float P_B = layer->base_pressure;
    const float T_B = layer->base_temperature;
    const float k   = layer->base_temperature_lapse_rate;

    // temperature
    result.temperature = T_B - k * (altitude - b);

    // static pressure, different barometric formulas for lapse rate / no lapse rate
    if (k == 0) {
        result.pressure = P_B * expf(-EARTH_G0 * (altitude - b) / (AIR_R * T_B));
    } else {
        result.pressure = P_B * powf(1.0f - (k / T_B) * (altitude - b), EARTH_G0 / (AIR_R * k));
    }

    // density
    result.density = result.pressure / (AIR_R * result.temperature);

    // local speed of sound
    result.mach_local = sqrtf(AIR_GAMMA * AIR_R * result.temperature);

    return result;
}