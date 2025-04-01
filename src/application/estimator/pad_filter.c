#include "application/estimator/pad_filter.h"
#include "FreeRTOS.h"
#include "common/math/math-algebra3d.h"
#include "math.h"
#include "model/model_airdata.h"

w_status_t pad_filter(estimator_all_imus_input_t *data) {
    // Computes inital state and covariance estimate for EKF, and bias values for the IMU
    // Uses all available sensors: Gyroscope W, Magnetometer M, Accelerometer A, Barometer P
    // Outputs: initial state, sensor bias matrix, [x_init, bias_1, bias_2, bias_3]

    // set constant initials

    const float alpha = 0.005; // low pass time constant

    const vector3d_t w = {0}; // stationary on rail
    const vector3d_t v = {0}; // stationary on rail

    const float Cl = 0;

    const float delta = 0; // controller sets canards to zero due to flight phase

    // global variable to select which IMUs are used
    // IMU_select[0] = polulu, IMU_select[1] = movella
    // TODO: Matlab code was a global variable, is it needed in C?
    bool IMU_select[2] = {data->polulu.is_dead, data->movella.is_dead};

    if (IMU_select[0] == false && IMU_select[1] == false) {
        return W_FAILURE; // no IMUs selected
    }

    // filtered_i is lowpass filtered data of IMU_i
    // TODO: do these arrays below remember from last iteration?

    // TODO: will be unnecessary after estimator_imu_measurement_t is changed
    float IMU_1[10] = {
        data->polulu.accelerometer.x,
        data->polulu.accelerometer.y,
        data->polulu.accelerometer.z,
        data->polulu.gyroscope.x,
        data->polulu.gyroscope.y,
        data->polulu.gyroscope.z,
        data->polulu.magnometer.x,
        data->polulu.magnometer.y,
        data->polulu.magnometer.z,
        data->polulu.barometer
    };

    float IMU_2[10] = {
        data->movella.accelerometer.x,
        data->movella.accelerometer.y,
        data->movella.accelerometer.z,
        data->movella.gyroscope.x,
        data->movella.gyroscope.y,
        data->movella.gyroscope.z,
        data->movella.magnometer.x,
        data->movella.magnometer.y,
        data->movella.magnometer.z,
        data->movella.barometer
    };

    float filtered_1[10];
    float filtered_2[10];

    // Ensure initializations runs only once
    static bool filtered_1_initialized = false;
    static bool filtered_2_initialized = false;

    // Initialization

    if (!filtered_1_initialized) {
        if (IMU_select[0] == true) { // if IMU_i alive
            memcpy(filtered_1, IMU_1, 10);
        } else {
            memset(filtered_1, 0, 10 * sizeof(float));
        }
        filtered_1_initialized = true;
    }

    if (!filtered_2_initialized) {
        if (IMU_select[1] == true) { // if IMU_i alive
            memcpy(filtered_2, IMU_2, 10);
        } else {
            memset(filtered_2, 0, 10 * sizeof(float));
        }
        filtered_2_initialized = true;
    }

    // lowpass filter

    // filtered = filtered + alpha * (measured - filtered);

    if (IMU_select[0] == true) {
        for (int i = 0; i < 10; i++) {
            filtered_1[i] = alpha * IMU_1[i] + (1 - alpha) * filtered_1[i];
        }
    }

    if (IMU_select[1] == true) {
        for (int i = 0; i < 10; i++) {
            filtered_2[i] = alpha * IMU_2[i] + (1 - alpha) * filtered_2[i];
        }
    }

    // State determination

    // average specific force of selected sensors
    float a[3] = {0}; // acceleration a

    if (IMU_select[0] == true) {
        for (int i = 0; i < 3; i++) {
            a[i] += filtered_1[i]; // only add alive IMUs to average
        }
    }

    if (IMU_select[1] == true) {
        for (int i = 0; i < 3; i++) {
            a[i] += filtered_2[i];
        }
    }

    if ((IMU_select[0] + IMU_select[1]) > 0) {
        for (int i = 0; i < 3; i++) {
            a[i] /= (IMU_select[0] + IMU_select[1]); // divide by number of alive IMUs
        }
    }

    // gravity vector in body - fixed frame

    float psi = atan2(-a[1], a[0]); // rail yaw angle
    float theta = atan2(a[2], a[0]); // rail pitch angle

    // compute launch attitude quaternion

    quaternion_t q = {
        {cos(psi / 2) * cos(theta / 2),
         -sin(psi / 2) * sin(theta / 2),
         cos(psi / 2) * sin(theta / 2),
         sin(psi / 2) * cos(theta / 2)}
    };

    // compute altitude

    float p = 0; // barometric pressure p

    if (IMU_select[0] == true) { // only add alive IMUs to average
        p += filtered_1[9];
    }

    if (IMU_select[1] == true) { // only add alive IMUs to average
        p += filtered_2[9];
    }

    if (IMU_select[0] + IMU_select[1] > 0) {
        p /= (IMU_select[0] + IMU_select[1]); // divide by number of alive IMUs
    }

    // current altitude

    float alt = model_altdata(p);

    // conconct state vector

    float x_init[13] = {q.w, q.x, q.y, q.z, w.x, w.y, w.z, v.x, v.y, v.z, alt, Cl, delta};

    // Bias determination

    // declare bias vectors
    float bias_1[10] = {0};
    float bias_2[10] = {0};

    // accelerometer
    // TODO: did not add accelerometer bias determination yet, leave out for now

    // gyroscope
    if (IMU_select[0] == true) {
        for (int i = 3; i <= 5; i++) {
            bias_1[i] = filtered_1[i];
        }
    }

    if (IMU_select[1] == true) {
        for (int i = 3; i <= 5; i++) {
            bias_2[i] = filtered_2[i];
        }
    }

    // earth magnetic field

    // TODO: quaternion_rotmatrix()?
    matrix3d_t ST = math_matrix3d_transp(quaternion_rotmatrix(q)); // launch attitude

    // Subset of the filtered data for magnetometer only
    vector3d_t filtered_1_magnetometer = {{filtered_1[6], filtered_1[7], filtered_1[8]}};
    vector3d_t filtered_2_magnetometer = {{filtered_2[6], filtered_2[7], filtered_2[8]}};

    vector3d_t bias_1_magnetometer = {0};
    vector3d_t bias_2_magnetometer = {0};

    if (IMU_select[0] == true) {
        bias_1_magnetometer = math_vector3d_rotate(&ST, &filtered_1_magnetometer);
        // TODO : add iron corrections. Maybe in IMU handler, next to rotation correction?
    }

    if (IMU_select[1] == true) {
        bias_2_magnetometer = math_vector3d_rotate(&ST, &filtered_2_magnetometer);
    }

    bias_1[6] = bias_1_magnetometer.x;
    bias_1[7] = bias_1_magnetometer.y;
    bias_1[8] = bias_1_magnetometer.z;

    bias_2[6] = bias_2_magnetometer.x;
    bias_2[7] = bias_2_magnetometer.y;
    bias_2[8] = bias_2_magnetometer.z;

    // barometer
    // TODO: barometer bias not yet implemented, leave out for now.
    // testing will show if we do pressure -> altitude, or if bias
    // correction is needed when pressure varies wildly. Could be
    // expected altitude on location) - (pressure -> altitude) -> (expected pressure)
    // (barometer bias) = (pressure) - (expected pressure)

    sensor_bias_t sensor_bias_output;
    x_init_t x_init_output;

    // Copy x_init values
    memcpy(x_init_output.x_init, x_init, 13 * sizeof(float));

    // Copy bias_1 values
    memcpy(sensor_bias_output.bias_1, bias_1, 10 * sizeof(float));

    // Copy bias_2 values
    memcpy(sensor_bias_output.bias_2, bias_2, 10 * sizeof(float));

    return W_SUCCESS;
}
