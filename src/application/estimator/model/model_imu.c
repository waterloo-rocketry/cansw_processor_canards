// include paths
#include "application/estimator/model/model_imu.h"
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_airdata.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"

// computes measurement prediction using current state and sensror biases
y_imu_t model_measurement_imu(const x_state_t *state, const y_imu_t *imu_bias) {
    y_imu_t measurement_prediction;

    // calculate rates
    const vector3d_t W = math_vector3d_add(&state->rates, &imu_bias->gyroscope); // W = w + b_W;

    // magnetic field model
    const matrix3d_t S = quaternion_rotmatrix(&state->attitude);
    const vector3d_t M = math_vector3d_rotate(&S, &imu_bias->magnetometer);

    // atmosphere model
    const estimator_airdata_t airdata = model_airdata(state->altitude);
    const double P = airdata.pressure;

    // final measurement prediction
    measurement_prediction.accelerometer = W;
    measurement_prediction.magnetometer = M;
    measurement_prediction.barometer = P;

    return measurement_prediction;
}

