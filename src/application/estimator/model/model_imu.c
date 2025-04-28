// include paths
#include "application/estimator/model/model_imu.h"
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/jacobians.h"
#include "application/estimator/model/model_airdata.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"

// computes measurement prediction using current state and sensror biases
y_imu_t model_measurement_imu(const x_state_t *state, const y_imu_t *imu_bias) {
    y_imu_t measurement_prediction = {0};

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

// DEV NOTES
// b_W: gyroscope
// m_E: magnetometer

// jacobian of the measurement model
void model_measurement_imu_jacobian(
    arm_matrix_instance_f64 *imu_jacobian, const x_state_t *state, const y_imu_t *imu_bias,
    double dt
) {
    // initialize
    measurement_model_jacobian_t *J = {0};

    // rates
    const matrix3d_t W_w = {.array = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}; 

    // magnetic field model
    rotation_jacobian_t M_q = {0};
    quaternion_rotate_jacobian(M_q.flat, &state->attitude, &imu_bias->magnetometer);

    // atmosphere model
    const double P_alt = model_airdata_jacobian(state->altitude);

    // measurement prediction
    write_pData(J.flat, 0, 4, SIDE_MATRIX_3D, SIDE_MATRIX_3D, &W_w.flat[0]);

    return ;
}