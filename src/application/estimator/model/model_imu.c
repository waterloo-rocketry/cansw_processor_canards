// include paths
#include "application/estimator/model/model_imu.h"
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_airdata.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"

// define constants (if applicable)

// define instances
x_state_t estimator_state __attribute__((unused)) = {0};
y_imu_t estimator_imu_data __attribute__((unused)) = {0};

// define functions

// computes measurement prediction using current state and sensror biases
y_imu_t model_measurement_imu_(x_state_t *est_state, y_imu_t *imu_bias) {
    y_imu_t measurement_prediction;

    // decompose the state vector
    const quaternion_t q = est_state->attitude;
    const vector3d_t w = est_state->rates;
    const double alt = est_state->altitude;

    // decompose the bias matrix
    const vector3d_t b_W = imu_bias->gyroscope;
    const vector3d_t m_E =
        imu_bias->magnetometer; // m_E instead of M_E cuz M_E is an existing variable in math.h

    // calculate rates
    const vector3d_t W = math_vector3d_add(&w, &b_W);

    // magnetic field model
    const matrix3d_t S = quaternion_rotmatrix(&q);
    const vector3d_t M = math_vector3d_rotate(&S, &m_E);

    // atmosphere model
    const estimator_airdata_t airdata = model_airdata(alt);
    const double P = airdata.pressure;

    // final measurement prediction
    measurement_prediction.accelerometer = W;
    measurement_prediction.magnetometer = M;
    measurement_prediction.barometer = P;

    return measurement_prediction;
}

