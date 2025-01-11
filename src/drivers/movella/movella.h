#ifndef MOVELLA_H
#define MOVELLA_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include "task.h"
#include "xsens_constants.h"
#include "xsens_mti.h"

typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct {
    uint32_t ns;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t flags;
} XsensUtcTime_t;

typedef struct {
    XsensUtcTime_t utc;
    Vector3f acc;
    Vector3f gyr;
    Vector3f euler;
    Vector3f mag;
    float pres;
    float temp;
    Quaternion quat;
} MovellaData_t;

bool movella_init(void);
void movella_reset_orientation(void);
bool movella_get_imu_axes(MovellaData_t *out_data);

bool movella_get_utc_time(XsensUtcTime_t *out_utc);
bool movella_get_acceleration(Vector3f *out_acc);
bool movella_get_rate_of_turn(Vector3f *out_gyro);
bool movella_get_orientation(Vector3f *out_euler);
bool movella_get_magnetometer(Vector3f *out_mag);
bool movella_get_pressure(float *out_pres);
bool movella_get_temperature(float *out_temp);
bool movella_get_quaternion(Quaternion *out_quat);

#endif
