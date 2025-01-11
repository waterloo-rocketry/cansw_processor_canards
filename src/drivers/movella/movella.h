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
    vector3d_t acc;
    vector3d_t gyr;
    vector3d_t euler;
    vector3d_t mag;
    float pres;
    float temp;
    quaternion_t quat;
} MovellaData_t;

w_status_t movella_init(void);
void movella_reset_orientation(void);
w_status_t movella_get_imu_axes(MovellaData_t *out_data);

w_status_t movella_get_utc_time(XsensUtcTime_t *out_utc);
w_status_t movella_get_acceleration(vector3d_t *out_acc);
w_status_t movella_get_rate_of_turn(vector3d_t *out_gyro);
w_status_t movella_get_orientation(vector3d_t *out_euler);
w_status_t movella_get_magnetometer(vector3d_t *out_mag);
w_status_t movella_get_pressure(float *out_pres);
w_status_t movella_get_temperature(float *out_temp);
w_status_t movella_get_quaternion(quaternion_t *out_quat);

#endif
