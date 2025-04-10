#include "application/estimator/pad_filter.h"
#include "application/estimator/estimator.h"
#include "application/estimator/model/model_airdata.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"

static const float low_pass_alpha = 0.005; // low pass time constant

// set constant initials - knowing that the rocket is stationary on the rail
static const vector3d_t w = {{0.0f}}; // stationary on rail
static const vector3d_t v = {{0.0f}}; // stationary on rail
static const float Cl = 5; // estimated coefficient of lift, const with Ma
static const float delta = 0; // controller sets canards to zero due to flight phase

// store the 1 pad filter context existing in this program
pad_filter_ctx_t g_pad_filter_ctx = {};

// Computes inital state and covariance estimate for EKF, and bias values for the IMU
// Uses all available sensors: Gyroscope W, Magnetometer M, Accelerometer A, Barometer P
// Outputs: initial state, sensor bias matrix, [x_init, bias_1, bias_2]
w_status_t pad_filter(
    pad_filter_ctx_t *ctx, const y_imu_t *IMU_1, const y_imu_t *IMU_2, const bool is_dead_1,
    const bool is_dead_2, x_state_t *x_init, y_imu_t *bias_1, y_imu_t *bias_2
) {
    if ((NULL == IMU_1) || (NULL == IMU_2) || (NULL == x_init) || (NULL == bias_1) ||
        (NULL == bias_2)) {
        return W_INVALID_PARAM;
    }

    // helper variables for easier access
    float *bias_1_arr = &bias_1->array[0];
    float *bias_2_arr = &bias_2->array[0];

    const float *IMU_1_arr = IMU_1->array;
    const float *IMU_2_arr = IMU_2->array;

    // select which IMUs are used based on current deadness
    const bool IMU_select[2] = {!is_dead_1, !is_dead_2};

    // trick to count the number of alive IMUs via imu select booleans
    uint32_t num_alive_imus = (uint32_t)IMU_select[0] + (uint32_t)IMU_select[1];

    // Failure if no IMUs selected, so don't need to check for division by 0 below
    if (num_alive_imus == 0) {
        return W_FAILURE;
    }

    // Initialization - only run 1 time in the whole program
    if (!ctx->filtered_1_initialized) {
        if (IMU_select[0]) { // if IMU_i alive
            memcpy(ctx->filtered_1_arr, IMU_1_arr, 10 * sizeof(float));
        } else {
            memset(ctx->filtered_1_arr, 0, 10 * sizeof(float));
        }
        ctx->filtered_1_initialized = true;
    }

    if (!ctx->filtered_2_initialized) {
        if (IMU_select[1]) { // if IMU_i alive
            memcpy(ctx->filtered_2_arr, IMU_2_arr, 10 * sizeof(float));
        } else {
            memset(ctx->filtered_2_arr, 0, 10 * sizeof(float));
        }
        ctx->filtered_2_initialized = true;
    }

    // lowpass filter

    // filtered = filtered + low_pass_alpha * (measured - filtered);

    if (IMU_select[0]) {
        for (int i = 0; i < 10; i++) {
            ctx->filtered_1_arr[i] =
                low_pass_alpha * IMU_1_arr[i] + (1 - low_pass_alpha) * ctx->filtered_1_arr[i];
        }
    }

    if (IMU_select[1]) {
        for (int i = 0; i < 10; i++) {
            ctx->filtered_2_arr[i] =
                low_pass_alpha * IMU_2_arr[i] + (1 - low_pass_alpha) * ctx->filtered_2_arr[i];
        }
    }

    // State determination

    // Average specific force of selected sensors

    // Sum accelerations from alive IMUs
    float a[3] = {0.0f, 0.0f, 0.0f}; // a[0] = x, a[1] = y, a[2] = z

    if (IMU_select[0]) { // if IMU_1 is alive
        a[0] += ctx->filtered_1_arr[0];
        a[1] += ctx->filtered_1_arr[1];
        a[2] += ctx->filtered_1_arr[2];
    }

    if (IMU_select[1]) { // if IMU_2 is alive
        a[0] += ctx->filtered_2_arr[0];
        a[1] += ctx->filtered_2_arr[1];
        a[2] += ctx->filtered_2_arr[2];
    }

    // Normalize the acceleration by the number of alive IMUs
    a[0] /= (float)num_alive_imus;
    a[1] /= (float)num_alive_imus;
    a[2] /= (float)num_alive_imus;

    // Gravity vector in body-fixed frame
    float psi = atan(-a[1] / a[0]);
    float theta = atan(a[2] / a[0]);

    // compute launch attitude quaternion

    quaternion_t q = {
        {cos(psi / 2.0f) * cos(theta / 2.0f),
         -sin(psi / 2.0f) * sin(theta / 2.0f),
         cos(psi / 2.0f) * sin(theta / 2.0f),
         sin(psi / 2.0f) * cos(theta / 2.0f)}
    };

    // compute altitude

    float p = 0; // barometric pressure p

    if (IMU_select[0]) { // only add alive IMUs to average
        p += ctx->filtered_1_arr[9];
    }

    if (IMU_select[1]) { // only add alive IMUs to average
        p += ctx->filtered_2_arr[9];
    }

    p /= (float)num_alive_imus;

    // current altitude

    const float alt = model_altdata(p);

    // conconct state vector
    x_init->attitude.w = q.w;
    x_init->attitude.x = q.x;
    x_init->attitude.y = q.y;
    x_init->attitude.z = q.z;
    x_init->rates.x = w.x;
    x_init->rates.y = w.y;
    x_init->rates.z = w.z;
    x_init->velocity.x = v.x;
    x_init->velocity.y = v.y;
    x_init->velocity.z = v.z;
    x_init->altitude = alt;
    x_init->CL = Cl;
    x_init->delta = delta;

    // Bias determination

    // accelerometer
    // TODO: did not add accelerometer bias determination yet, leave out for now

    // gyroscope
    if (IMU_select[0]) {
        for (int i = 3; i <= 5; i++) {
            bias_1_arr[i] = ctx->filtered_1_arr[i];
        }
    }

    if (IMU_select[1]) {
        for (int i = 3; i <= 5; i++) {
            bias_2_arr[i] = ctx->filtered_2_arr[i];
        }
    }

    // Earth magnetic field
    matrix3d_t R = quaternion_rotmatrix(&q); // Get rotation matrix from quaternion
    matrix3d_t ST = math_matrix3d_transp(&R); // Transpose of that rotation matrix

    if (IMU_select[0] == 1) { // if IMU_1 is alive
        // Rotate the magnetic field using the quaternion rotation matrix ST
        vector3d_t mag_rotated_1 = math_vector3d_rotate(&ST, (vector3d_t *)&ctx->filtered_1_arr[6]);

        // Update the bias_1 array with the rotated magnetic field values
        bias_1->array[6] = mag_rotated_1.x;
        bias_1->array[7] = mag_rotated_1.y;
        bias_1->array[8] = mag_rotated_1.z;
    }

    if (IMU_select[1] == 1) { // if IMU_2 is alive
        // Rotate the magnetic field using the quaternion rotation matrix ST
        vector3d_t mag_rotated_2 = math_vector3d_rotate(&ST, (vector3d_t *)&ctx->filtered_2_arr[6]);

        // Update the bias_2 array with the rotated magnetic field values
        bias_2->array[6] = mag_rotated_2.x;
        bias_2->array[7] = mag_rotated_2.y;
        bias_2->array[8] = mag_rotated_2.z;
    }

    // Barometer

    // TODO: barometer bias not yet implemented, leave out for now.
    // testing will show if we do pressure -> altitude, or if bias
    // correction is needed when pressure varies wildly. Could be
    // expected altitude on location) - (pressure -> altitude) -> (expected pressure)
    // (barometer bias) = (pressure) - (expected pressure)

    return W_SUCCESS;
}
