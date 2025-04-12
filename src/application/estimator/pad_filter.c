/**
 * everything in this file follows simulink-canards commit e20e5d1
 */

#include "application/estimator/pad_filter.h"
#include "application/estimator/estimator.h"
#include "application/estimator/model/model_airdata.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"

static const double low_pass_alpha = 0.005; // low pass time constant

// set constant initials - knowing that the rocket is stationary on the rail
static const vector3d_t w = {{0.0}}; // stationary on rail
static const vector3d_t v = {{0.0}}; // stationary on rail
static const double Cl = 5; // estimated coefficient of lift, const with Ma
static const double delta = 0; // controller sets canards to zero due to flight phase

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

    // select which IMUs are used based on current deadness
    const bool IMU_select[2] = {!is_dead_1, !is_dead_2};

    // trick to count the number of alive IMUs via imu select booleans
    uint32_t num_alive_imus = (uint32_t)IMU_select[0] + (uint32_t)IMU_select[1];

    // Failure if no IMUs selected, so don't need to check for division by 0 below
    if (num_alive_imus == 0) {
        return W_FAILURE;
    }

    // Initialization - only run 1 time in the whole program
    // TODO: could optimize init to be done once outside of pad_filter?
    if (!ctx->filtered_1_initialized) {
        if (IMU_select[0]) { // if IMU_i alive
            memcpy(&ctx->filtered_1, IMU_1, sizeof(y_imu_t));
        } else {
            memset(&ctx->filtered_1, 0, sizeof(y_imu_t));
        }
        ctx->filtered_1_initialized = true;
    }

    if (!ctx->filtered_2_initialized) {
        if (IMU_select[1]) { // if IMU_i alive
            memcpy(&ctx->filtered_2, IMU_2, sizeof(y_imu_t));
        } else {
            memset(&ctx->filtered_2, 0, sizeof(y_imu_t));
        }
        ctx->filtered_2_initialized = true;
    }

    // lowpass filter

    // filtered = filtered + low_pass_alpha * (measured - filtered);

    if (IMU_select[0]) {
        for (int i = 0; i < Y_IMU_SIZE_FLOATS; i++) {
            ctx->filtered_1.array[i] =
                low_pass_alpha * IMU_1->array[i] + (1 - low_pass_alpha) * ctx->filtered_1.array[i];
        }
    }

    if (IMU_select[1]) {
        for (int i = 0; i < Y_IMU_SIZE_FLOATS; i++) {
            ctx->filtered_2.array[i] =
                low_pass_alpha * IMU_2->array[i] + (1 - low_pass_alpha) * ctx->filtered_2.array[i];
        }
    }

    // State determination

    // Average specific force of selected sensors

    // Sum accelerations from alive IMUs
    vector3d_t a = {.array = {0.0, 0.0, 0.0}};

    if (IMU_select[0]) { // if IMU_1 is alive
        a = math_vector3d_add(&a, &ctx->filtered_1.accelerometer);
    }

    if (IMU_select[1]) { // if IMU_2 is alive
        a = math_vector3d_add(&a, &ctx->filtered_2.accelerometer);
    }

    // Normalize the acceleration by the number of alive IMUs
    a = math_vector3d_scale(1.0 / (double)num_alive_imus, &a);

    // Gravity vector in body-fixed frame
    double psi = atan(-a.y / a.x);
    double theta = atan(a.z / a.x);

    // compute launch attitude quaternion

    quaternion_t q = {
        {cos(psi / 2.0) * cos(theta / 2.0),
         -sin(psi / 2.0) * sin(theta / 2.0),
         cos(psi / 2.0) * sin(theta / 2.0),
         sin(psi / 2.0) * cos(theta / 2.0)}
    };

    // compute altitude

    double p = 0; // barometric pressure p

    if (IMU_select[0]) { // only add alive IMUs to average
        p += ctx->filtered_1.barometer;
    }

    if (IMU_select[1]) { // only add alive IMUs to average
        p += ctx->filtered_2.barometer;
    }

    p /= (double)num_alive_imus;

    // current altitude

    const double alt = model_altdata(p);

    // concoct state vector. use compound literal syntax for convenience
    *x_init = (x_state_t
    ){.attitude = q, .rates = w, .velocity = v, .altitude = alt, .CL = Cl, .delta = delta};

    // Bias determination

    // accelerometer
    // TODO: did not add accelerometer bias determination yet, leave out for now

    // gyroscope
    if (IMU_select[0]) {
        bias_1->gyroscope = ctx->filtered_1.gyroscope;
    }

    if (IMU_select[1]) {
        bias_2->gyroscope = ctx->filtered_2.gyroscope;
    }

    // Earth magnetic field
    matrix3d_t R = quaternion_rotmatrix(&q); // Get rotation matrix from quaternion
    matrix3d_t ST = math_matrix3d_transp(&R); // Transpose of that rotation matrix

    if (IMU_select[0] == 1) { // if IMU_1 is alive
        // Rotate the magnetic field using the quaternion rotation matrix ST
        bias_1->magnetometer = math_vector3d_rotate(&ST, &ctx->filtered_1.magnetometer);
    }

    if (IMU_select[1] == 1) { // if IMU_2 is alive
        // Rotate the magnetic field using the quaternion rotation matrix ST
        bias_2->magnetometer = math_vector3d_rotate(&ST, &ctx->filtered_2.magnetometer);
    }

    // Barometer

    // TODO: barometer bias not yet implemented, leave out for now.
    // testing will show if we do pressure -> altitude, or if bias
    // correction is needed when pressure varies wildly. Could be
    // expected altitude on location) - (pressure -> altitude) -> (expected pressure)
    // (barometer bias) = (pressure) - (expected pressure)

    return W_SUCCESS;
}
