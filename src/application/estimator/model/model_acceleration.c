#include <stdbool.h>
#include <stdint.h>

#include "application/estimator/model/model_acceleration.h"
#include "common/math/math-algebra3d.h"

// from params.m commit 6eb11c1 - for testflight
// positions of IMU centers relative to body frame
static const vector3d_t d1 = {{-0.127, -0.074, 0.027}};
static const vector3d_t d2 = {{-0.127, -0.065, -0.047}};

vector3d_t model_acceleration(
    const x_state_t *state, const y_imu_t *IMU_1, const bool is_dead_1, const y_imu_t *IMU_2,
    const bool is_dead_2
) {
    vector3d_t a = {{0.0, 0.0, 0.0}};
    const vector3d_t w = state->rates;
    uint32_t num_alive_imus = 0;

    if (!is_dead_1) {
        vector3d_t w_cross_d1 = math_vector3d_cross(&w, &d1);
        vector3d_t correction_1 = math_vector3d_cross(&w, &w_cross_d1);
        vector3d_t corrected_imu1 = math_vector3d_subt(&IMU_1->accelerometer, &correction_1);

        a = math_vector3d_add(&a, &corrected_imu1);
        num_alive_imus += 1;
    }

    if (!is_dead_2) {
        vector3d_t w_cross_d2 = math_vector3d_cross(&w, &d2);
        vector3d_t correction_2 = math_vector3d_cross(&w, &w_cross_d2);
        vector3d_t corrected_imu2 = math_vector3d_subt(&IMU_2->accelerometer, &correction_2);

        a = math_vector3d_add(&a, &corrected_imu2);
        num_alive_imus += 1;
    }

    if (num_alive_imus > 0) {
        a = math_vector3d_scale(1.0 / (double)num_alive_imus, &a);
    }

    return a;
}
