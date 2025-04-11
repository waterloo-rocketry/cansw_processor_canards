// include paths
#include "application/estimator/model/model_imu.h"
#include "application/estimator/model/model_airdata.h"
#include "application/estimator/model/quaternion.h"

// define constants (if applicable)

// define instances
est_state_t estimator_state __attribute__((unused)) = {0};
est_imu_data_t estimator_imu_data __attribute__((unused)) = {0};

// define functions (helper functions to do vector and matrix math)

// add 2 vector3d_t's
vector3d_t vector3d_add(vector3d_t a, vector3d_t b) {
    vector3d_t result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

// perform matrix-vector mutliplication for a 3x3 matrix and a 3x1 vector
vector3d_t matrix3d_vector3d_multiply(matrix3d_t matrix, vector3d_t vector) {
    vector3d_t result;
    result.x = matrix.s11 * vector.x + matrix.s12 * vector.y + matrix.s13 * vector.z;
    result.y = matrix.s21 * vector.x + matrix.s22 * vector.y + matrix.s23 * vector.z;
    result.z = matrix.s31 * vector.x + matrix.s32 * vector.y + matrix.s33 * vector.z;
    return result;
}

est_imu_data_t model_measurement_imu(est_state_t *est_state, est_imu_data_t *imu_bias) {
    est_imu_data_t measurement_prediction;

    // decompose the state vector
    quaternion_t q = est_state->attitude;
    vector3d_t w = est_state->rates;
    // vector3d_t v = est_state->velocity; // not acutally used
    float alt = est_state->altitude;
    // float CL = est_state->CL; // not actually used
    // float delta = est_state->delta; // not actually used

    // decompose the bias matrix
    vector3d_t b_W = imu_bias->gyroscope;
    vector3d_t m_E = imu_bias->magnetometer; // m_E instead of M_E cuz M_E is an existing variable in math.h

    // calculate rates
    vector3d_t W = vector3d_add(w, b_W);

    // magnetic field model
    matrix3d_t S = quaternion_rotmatrix(&q);
    vector3d_t M = matrix3d_vector3d_multiply(S, m_E);

    // atmosphere model
    estimator_airdata_t airdata = model_airdata(alt);
    float P = airdata.pressure;

    // final measurement prediction
    measurement_prediction.accelerometer = W;
    measurement_prediction.magnetometer = M;
    measurement_prediction.barometer = P;

    return measurement_prediction;
}

