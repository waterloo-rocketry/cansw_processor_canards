
#include "application/estimator/ekf.h"
#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/model/model_imu.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <math.h>
#include <stdint.h>

/*
 * Filter settings --------------------------------
 */
#define SIZE_VECTOR_MAX 13 // set this to the maximum size of any vector here
#define SIZE_ACCELERATION 3
#define SIZE_IMU_MTI 10
#define SIZE_IMU_ALTIMU 10

#define MS_TO_SECOND 1000.0f

// Weighting, measurement model: MTi630
static double R_MTI_diag[SIZE_IMU_ALTIMU - SIZE_ACCELERATION] = {
    // Gyro,           Mag,              Baro
    1e-5,
    1e-5,
    1e-5,
    5e-3,
    5e-3,
    5e-3,
    2e1
};

// Weighting, measurement model: Polulu AltIMU v6
static double R_ALTIMU_diag[SIZE_IMU_ALTIMU - SIZE_ACCELERATION] = {
    // Gyro,           Mag,              Baro
    2e-5,
    2e-5,
    2e-5,
    1e-3,
    1e-3,
    1e-3,
    3e1
};

/*
 * Matrix memory space allocations --------------------------------
 */
static double P_data[X_STATE_SIZE_ITEMS * X_STATE_SIZE_ITEMS];
static double K_data[X_STATE_SIZE_ITEMS * SIZE_VECTOR_MAX];
static double buffer1[SIZE_VECTOR_MAX * SIZE_VECTOR_MAX];
static double buffer2[SIZE_VECTOR_MAX * SIZE_VECTOR_MAX];
static double buffer3[SIZE_VECTOR_MAX * SIZE_VECTOR_MAX];
static double buffer4[SIZE_VECTOR_MAX * SIZE_VECTOR_MAX];

static arm_matrix_instance_f64 Q;

static arm_matrix_instance_f64 P;
static const arm_matrix_instance_f64 K;
static arm_matrix_instance_f64 Q_dt; // dt* Q
static arm_matrix_instance_f64 R_MTI;

/**
 * @example Q
 */
void ekf_init(double dt) {
    // Weighting, dynamics model
    const double Q_diag[X_STATE_SIZE_ITEMS] = {
        1e-8, 1e-8, 1e-8, 1e-8, 1e0, 1e0, 1e0, 2e-2, 2e-2, 2e-2, 1e-2, 100, 10
    };
    math_init_matrix_diag(&Q, (uint16_t)X_STATE_SIZE_ITEMS, Q_diag);

    // Weighting, measurement model: Movella MTI630
    const double R_MTI_diag[SIZE_IMU_MTI - SIZE_ACCELERATION] = {
        // Gyro,           Mag,              Baro
        1e-5,
        1e-5,
        1e-5,
        5e-2,
        5e-2,
        5e-2,
        20
    };
    math_init_matrix_diag(&R_MTI, (uint16_t)SIZE_IMU_MTI, R_MTI_diag);
    // TODO other weighting matrices too...
}

/*
 * Algorithms --------------------------------
 */
void ekf_algorithm(
    x_state_t *state, const u_dynamics_t *input, const y_imu_t *imu_mti, const y_imu_t *bias_mti,
    const y_imu_t *imu_altimu, const y_imu_t *bias_altimu, const double *encoder, double dt, const bool in_dead_MTI, const bool is_dead_ALTIMU
) {
    // Predict
    x_state_t state_predict = {0};
    arm_matrix_instance_f64 P_predict = {0};
    ekf_matrix_predict(&state_predict, &P_predict, state, input, dt);
    *state = state_predict;

    // Correct (for one IMU)
    const double R = 0.01;
    ekf_matrix_correct(&P, const arm_matrix_instance_f64 *R, const uint16_t size_measurement,
        const x_state_t *state, double encoder
    )
    // add if(not dead)

    arm_matrix_instance_f64 H_x = model_meas_mti_jacobian(state, bias_mti);
    ekf_matrix_correct(&P, &K, &H_x, &R_MTI, SIZE_IMU_MTI);
    y_imu_t h_x = model_meas_mti_pred(state, bias_mti);
    double innovation[SIZE_IMU_MTI];
    arm_sub_f64(imu_mti->array, h_x.array, innovation, SIZE_IMU_MTI);
    double state_difference[X_STATE_SIZE_ITEMS];
    arm_mat_vec_mult_f64(&K, innovation, state_difference);
    arm_add_f64(state, state_difference, state_new.array, X_STATE_SIZE_ITEMS);
    // state = state_new (but with pointers?)

    // repeat Correct for different sensors: IMU and Encoder
}

void ekf_matrix_predict(
    x_state_t *x_new, arm_matrix_instance_f64 *P_new, const x_state_t *state,
    const u_dynamics_t *input, double dt
) {
    // DISCRETE DYANMICS UPDATE
    *x_new = model_dynamics_update(state, input, dt);

    // discrete jacobian: F = df/dx
    arm_matrix_instance_f64 F = {0};
    model_dynamics_jacobian(&F, state, input, dt);

    // DISCRETE COVARIANCE
    // compute F'
    arm_matrix_instance_f64 F_transp = {
        .numCols = X_STATE_SIZE_ITEMS, .numRows = X_STATE_SIZE_ITEMS, .pData = (0)
    };
    arm_mat_trans_f64(&F, &F_transp);

    // FP = F*P
    arm_matrix_instance_f64 FP = {
        .numCols = X_STATE_SIZE_ITEMS, .numRows = X_STATE_SIZE_ITEMS, .pData = (0)
    };
    arm_mat_mult_f64(&F, &P, &FP);

    // F * P * F'
    arm_matrix_instance_f64 FPF_transp = {
        .numCols = X_STATE_SIZE_ITEMS, .numRows = X_STATE_SIZE_ITEMS, .pData = (0)
    };
    arm_mat_mult_f64(&FP, &F_transp, &FPF_transp);

    // P_new = FPF' + dt * Q
    P_new->numCols = X_STATE_SIZE_ITEMS;
    P_new->numRows = X_STATE_SIZE_ITEMS;
    arm_mat_add_f32(&FPF_transp, &Q_dt, &P_new);
}

// needs x, R, encoder
void ekf_matrix_correct(
    arm_matrix_instance_f64 *P, const arm_matrix_instance_f64 *R, const uint16_t size_measurement,
    const x_state_t *state, double encoder
) {
    // CORRECTION
    // compute expected measurement and difference to measured values

    // const double y_expected_scalar = model_meas_encoder(state);
    // const double innovation_scalar = encoder - y_expected_scalar;
    const double innovation_scalar = encoder;
    const double innovation[X_STATE_SIZE_ITEMS] = {0};
    memset(innovation, innovation_scalar, X_STATE_SIZE_ITEMS); // convert scalars into vectors

    // TODO compute Jacobian: H = dh/dx
    // const x_state_t H_state = model_meas_encoder_jacobian();
    const arm_matrix_instance_f64 H;

    // compute Kalman gain (and helper matrices)
    // H' = trans(H) // b1
    arm_matrix_instance_f64 H_transp = {
        .numCols = size_measurement, .numRows = X_STATE_SIZE_ITEMS, .pData = (0)
    };
    arm_mat_trans_f64(&H, &H_transp);

    // PH' = P * H' // b2
    arm_matrix_instance_f64 PH_transp = {
        .numRows = X_STATE_SIZE_ITEMS, .numCols = size_measurement, .pData = (0)
    };
    arm_mat_mult_f64(P, &H_transp, &PH_transp);

    // HPH' = H * PH' // b3
    arm_matrix_instance_f64 HPH_transp = {
        .numCols = size_measurement, .numRows = size_measurement, .pData = (0)
    };
    arm_mat_mult_f64(&H, &PH_transp, &HPH_transp);

    // L = HPH' + R // b1
    arm_matrix_instance_f64 L = {
        .numCols = size_measurement, .numRows = size_measurement, .pData = (0)
    };
    arm_mat_add_f64(&HPH_transp, R, &L);

    // Linv = inv(L) // b3
    arm_matrix_instance_f64 L_inv = {
        .numCols = size_measurement, .numRows = size_measurement, .pData = (0)
    };
    math_status = arm_mat_inverse_f64(&L, &L_inv);

    // Kalman gain
    // K =  PH' * inv(L) // K_data
    arm_matrix_instance_f64 K = {
        .numRows = X_STATE_SIZE_ITEMS, .numCols = size_measurement, .pData = K_data
    };
    arm_mat_add_f64(&PH_transp, &L_inv, &K);

    // KH = K*H // b3
    arm_matrix_instance_f64 KH = {
        .numRows = X_STATE_SIZE_ITEMS, .numCols = size_measurement, .pData = (0)
    };
    arm_mat_mult_f64(&K, &H, &KH);

    // // I = eye // I_data
    arm_matrix_instance_f64 I;
    math_init_matrix_identity(&I, X_STATE_SIZE_ITEMS);

    // E = I - KH // b2
    arm_matrix_instance_f64 E = {
        .numRows = X_STATE_SIZE_ITEMS, .numCols = X_STATE_SIZE_ITEMS, .pData = (0)
    };
    arm_mat_sub_f64(&I, &KH, &E);

    // E' = trans(E) // b1
    arm_matrix_instance_f64 E_transp = {
        .numCols = X_STATE_SIZE_ITEMS, .numRows = X_STATE_SIZE_ITEMS, .pData = (0)
    };
    arm_mat_trans_f64(&E, &E_transp);

    // PE' = P*E' // b3
    arm_matrix_instance_f64 PE_transp = {
        .numCols = X_STATE_SIZE_ITEMS, .numRows = X_STATE_SIZE_ITEMS, .pData = (0)
    };
    arm_mat_mult_f64(P, &E_transp, &PE_transp);

    // EPE' = E*PE' // b1
    arm_matrix_instance_f64 EPE_transp = {
        .numCols = X_STATE_SIZE_ITEMS, .numRows = X_STATE_SIZE_ITEMS, .pData = (0)
    };
    arm_mat_mult_f64(&E, &PE_transp, &EPE_transp);

    // K_transp = trans(K) // b2
    arm_matrix_instance_f64 K_transp = {
        .numCols = X_STATE_SIZE_ITEMS, .numRows = size_measurement, .pData = (0)
    };
    arm_mat_trans_f64(&K, &K_transp);

    // RK' = R*K' // b3
    arm_matrix_instance_f64 RK_transp = {
        .numRows = size_measurement, .numCols = X_STATE_SIZE_ITEMS, .pData = (0)
    };
    arm_mat_mult_f64(R, &K_transp, &RK_transp);

    // KRK' = K*RK' // b2
    arm_matrix_instance_f64 KRK_transp = {
        .numRows = X_STATE_SIZE_ITEMS, .numCols = X_STATE_SIZE_ITEMS, .pData = (0)
    };
    arm_mat_mult_f64(&K, &RK_transp, &KRK_transp);

    // P_new = EPE' + KRK' // b3
    arm_matrix_instance_f64 P_new = {
        .numRows = X_STATE_SIZE_ITEMS, .numCols = X_STATE_SIZE_ITEMS, .pData = (0)
    };
    arm_mat_add_f64(&EPE_transp, &KRK_transp, &P_new);

    // TODO current state estimate
    x_state_t x_new = {0};
    arm_mat_vec_mult_f32(&K, &innovation[0], &x_new.array[0]); // K * innovation
    arm_add_f64(state, &x_new.array[0], &x_new.array[0], X_STATE_SIZE_ITEMS); // x + K * innovation

    const quaternion_t x_new_q = x_new.attitude; // normalize attitude quaternion
    x_new.attitude = quaternion_normalize(&x_new_q);
}

