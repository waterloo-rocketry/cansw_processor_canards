#include "application/estimator/estimator.h"
#include "FreeRTOS.h"

// Stub implementation
w_status_t estimator_init() {
    return W_SUCCESS;
}

w_status_t estimator_update_inputs_imu(estimator_all_imus_input_t *data) {
    (void)data; // Explicitly mark parameter as unused
    // Just a stub that accepts data but does nothing with it
    return W_SUCCESS;
}

w_status_t pad_filter(estimator_all_imus_input_t *data) {
    // Computes inital state and covariance estimate for EKF, and bias values for the IMU
    // Uses all available sensors: Gyroscope W, Magnetometer M, Accelerometer A, Barometer P
    // Outputs: initial state, sensor bias matrix, [x_init, bias_1, bias_2, bias_3]

    // global variable to select which IMUs are used
    // IMU_select[0] = polulu, IMU_select[1] = movella
    // TODO: Check if this is the correct way and place to do this in C, global?
    bool IMU_select[2] = {data->polulu.is_dead, data->movella.is_dead};

    // filtered_i is lowpass filtered data of IMU_i
    // TODO: remembers from last iteration doesn't work with static variables

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

    static bool filtered_1_initialized = false; // Ensure this initialization runs only once
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

    const float alpha = 0.005;

    // low pass time constant
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

    if (IMU_select[0] == 1) {
        for (int i = 0; i < 3; i++) {
            a[i] += filtered_1[i]; // only add alive IMUs to average
        }
    }

    if (IMU_select[1] == 1) {
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

    float psi = atan(-a[1] / a[0]); // rail yaw angle
    float theta = atan(a[2] / a[0]); // rail pitch angle

    // compute launch attitude quaternion

    float q[4] = {
        cos(psi / 2) * cos(theta / 2),
        -sin(psi / 2) * sin(theta / 2),
        cos(psi / 2) * sin(theta / 2),
        sin(psi / 2) * cos(theta / 2)
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

    // set constant initials

    float w[3] = {0}; // stationary on rail
    float v[3] = {0}; // stationary on rail

    float Cl = 0;

    float delta = 0; // controller sets canards to zero due to flight phase

    // conconct state vector

    float x_init[13] = {q[0], q[1], q[2], q[3], w[0], w[1], w[2], v[0], v[1], v[2], alt, Cl, delta};

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

    float ST = transpose(quaternion_rotmatrix(q)); // launch attitude

    if (IMU_select[0] == true) {
        for (int i = 6; i <= 8; i++) {
            bias_1[i] = ST * filtered_1[i];
            // TODO : add iron corrections.Maybe in IMU handler, next to rotation correction?
        }
    }

    if (IMU_select[1] == true) {
        for (int i = 6; i <= 8; i++) {
            bias_2[i] = ST * filtered_2[i];
        }
    }

    // barometer
    // barometer bias not yet implemented, leave out for now.
    // testing will show if we do pressure -> altitude, or if bias
    // correction is needed when pressure varies wildly. Could be
    // expected altitude on location) - (pressure -> altitude) -> (expected pressure)
    // (barometer bias) = (pressure) - (expected pressure)

    sensor_bias_t sensor_bias;
    // Copy x_init values
    memcpy(sensor_bias.x_init, x_init, 13);

    // Copy bias_1 values
    memcpy(sensor_bias.bias_1, bias_1, 10);

    // Copy bias_2 values
    memcpy(sensor_bias.bias_2, bias_2, 10);

    return W_SUCCESS;
}
