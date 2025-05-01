
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

/*
 * Matrix memory space allocations --------------------------------
 */

static arm_matrix_instance_f64 Q_dt; // dt* Q
static arm_matrix_instance_f64 R_MTI;
static arm_matrix_instance_f64 R_ALTIMU;

/**
 * @example Q_dt
 * @example R_MTI
 * @example R_ALTIMU
 */
void ekf_init(double dt) {
    // Weighting, dynamics model
    double Q_diag[SIZE_STATE] = {
        1e-8, 1e-8, 1e-8, 1e-8, 1e0, 1e0, 1e0, 2e-2, 2e-2, 2e-2, 1e-2, 100, 10
    };
    for (int i = 0; i < SIZE_STATE; i++) {
        Q_diag[i] *= dt;
    }
    math_init_matrix_diag(&Q_dt, (uint16_t)SIZE_STATE, Q_diag);

    // Weighting, measurement model: MTi630
    const double R_MTI_diag[SIZE_IMU_MEAS] = {
        // Gyro,           Mag, Baro
        1e-5,
        1e-5,
        1e-5,
        5e-3,
        5e-3,
        5e-3,
        2e1
    };
    math_init_matrix_diag(&R_MTI, (uint16_t)SIZE_IMU_MEAS, R_MTI_diag);

    // Weighting, measurement model: Polulu AltIMU v6
    const double R_ALTIMU_diag[SIZE_IMU_MEAS] = {
        // Gyro,           Mag, Baro
        2e-5,
        2e-5,
        2e-5,
        1e-3,
        1e-3,
        1e-3,
        3e1
    };
    math_init_matrix_diag(&R_ALTIMU, (uint16_t)SIZE_IMU_MEAS, R_ALTIMU_diag);

    // TODO other weighting matrices too...
}

/*
 * Algorithms --------------------------------
 */
void ekf_algorithm(
    x_state_t *state, arm_matrix_instance_f64 *P, const u_dynamics_t *input, const y_imu_t *imu_mti,
    const y_imu_t *bias_mti, const y_imu_t *imu_altimu, const y_imu_t *bias_altimu, double encoder,
    double dt, const bool is_dead_MTI, const bool is_dead_ALTIMU
) {
    // init all matrices
    ekf_init(dt);

    // Predict
    x_state_t state_predict = {0};
    arm_matrix_instance_f64 P_predict = {0};
    ekf_matrix_predict(state, &P_predict, input, dt);
    *state = state_predict;

    // Correct (for one IMU)

    // add if(not dead)

    if (!is_dead_MTI) {
        ekf_matrix_correct(state, P, &R_MTI, SIZE_STATE, imu_mti, bias_mti);
    }

    if (!is_dead_ALTIMU) {
        ekf_matrix_correct(state, P, &R_ALTIMU, SIZE_STATE, imu_altimu, bias_altimu);
    }

    // arm_matrix_instance_f64 H_x = model_meas_mti_jacobian(state, bias_mti);
    // ekf_matrix_correct(&P, &K, &H_x, &R_MTI, SIZE_IMU_MEAS);
    // y_imu_t h_x = model_meas_mti_pred(state, bias_mti);
    // double innovation[SIZE_IMU_MEAS];
    // arm_sub_f64(imu_mti->array, h_x.array, innovation, SIZE_IMU_MEAS);
    // double state_difference[SIZE_STATE];
    // arm_mat_vec_mult_f64(&K, innovation, state_difference);
    // arm_add_f64(state, state_difference, state_new.array, SIZE_STATE);
    // state = state_new (but with pointers?)

    // repeat Correct for different sensors: IMU and Encoder
}

void ekf_matrix_predict(
    x_state_t *state, arm_matrix_instance_f64 *P, const u_dynamics_t *input, double dt
) {
    // DISCRETE DYANMICS UPDATE
    *state = model_dynamics_update(state, input, dt);

    // discrete jacobian: F = df/dx
    arm_matrix_instance_f64 F = {0};
    model_dynamics_jacobian(&F, state, input, dt);

    // DISCRETE COVARIANCE
    // compute F'
    arm_matrix_instance_f64 F_transp = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = (0)};
    arm_mat_trans_f64(&F, &F_transp);

    // FP = F*P
    arm_matrix_instance_f64 FP = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = (0)};
    arm_mat_mult_f64(&F, P, &FP);

    // F * P * F'
    arm_matrix_instance_f64 FPF_transp = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = (0)
    };
    arm_mat_mult_f64(&FP, &F_transp, &FPF_transp);

    // P_new = FPF' + dt * Q

    const arm_matrix_instance_f32 *FPF_transp_const = (arm_matrix_instance_f32 *)&FPF_transp;
    arm_mat_add_f32(
        FPF_transp_const, (arm_matrix_instance_f32 *)&Q_dt, (arm_matrix_instance_f32 *)P
    );
}

// needs y_imu_t as bias
// corrected state where
void ekf_matrix_correct(
    x_state_t *state, arm_matrix_instance_f64 *P, const arm_matrix_instance_f64 *R,
    const uint16_t size_measurement, const y_imu_t *imu, const y_imu_t *bias
) {
    // CORRECTION
    // compute expected measurement and difference to measured values

    const y_imu_t y_expected = model_measurement_imu(state, bias);
    const double y[SIZE_IMU_MEAS]; // imu_1(4:end)
    double innovation[SIZE_IMU_MEAS] = {0};
    arm_sub_f64(
        (float64_t *)&y, (float64_t *)&y_expected.array[0], (float64_t *)&innovation, SIZE_IMU_MEAS
    );

    // TODO compute Jacobian: H = dh/dx
    // const x_state_t H_state = model_meas_encoder_jacobian();
    const arm_matrix_instance_f64 H;

    // compute Kalman gain (and helper matrices)
    // H' = trans(H) // b1
    arm_matrix_instance_f64 H_transp = {
        .numCols = size_measurement, .numRows = SIZE_STATE, .pData = (0)
    };
    arm_mat_trans_f64(&H, &H_transp);

    // PH' = P * H' // b2
    arm_matrix_instance_f64 PH_transp = {
        .numRows = SIZE_STATE, .numCols = size_measurement, .pData = (0)
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
    arm_mat_add_f32(
        (arm_matrix_instance_f32 *)&HPH_transp,
        (arm_matrix_instance_f32 *)R,
        (arm_matrix_instance_f32 *)&L
    );

    // Linv = inv(L) // b3
    arm_matrix_instance_f64 L_inv = {
        .numCols = size_measurement, .numRows = size_measurement, .pData = (0)
    };
    arm_mat_inverse_f64(&L, &L_inv);

    // Kalman gain
    // K =  PH' * inv(L) // K_data
    arm_matrix_instance_f64 K = {.numRows = SIZE_STATE, .numCols = size_measurement, .pData = (0)};
    arm_mat_add_f32(
        (arm_matrix_instance_f32 *)&PH_transp,
        (arm_matrix_instance_f32 *)&L_inv,
        (arm_matrix_instance_f32 *)&K
    );

    // KH = K*H // b3
    arm_matrix_instance_f64 KH = {.numRows = SIZE_STATE, .numCols = size_measurement, .pData = (0)};
    arm_mat_mult_f64(&K, &H, &KH);

    // // I = eye // I_data
    arm_matrix_instance_f64 I;
    math_init_matrix_identity(&I, SIZE_STATE);

    // E = I - KH // b2
    arm_matrix_instance_f64 E = {.numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = (0)};
    arm_mat_sub_f64(&I, &KH, &E);

    // E' = trans(E) // b1
    arm_matrix_instance_f64 E_transp = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = (0)};
    arm_mat_trans_f64(&E, &E_transp);

    // PE' = P*E' // b3
    arm_matrix_instance_f64 PE_transp = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = (0)
    };
    arm_mat_mult_f64(P, &E_transp, &PE_transp);

    // EPE' = E*PE' // b1
    arm_matrix_instance_f64 EPE_transp = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = (0)
    };
    arm_mat_mult_f64(&E, &PE_transp, &EPE_transp);

    // K_transp = trans(K) // b2
    arm_matrix_instance_f64 K_transp = {
        .numCols = SIZE_STATE, .numRows = size_measurement, .pData = (0)
    };
    arm_mat_trans_f64(&K, &K_transp);

    // RK' = R*K' // b3
    arm_matrix_instance_f64 RK_transp = {
        .numRows = size_measurement, .numCols = SIZE_STATE, .pData = (0)
    };
    arm_mat_mult_f64(R, &K_transp, &RK_transp);

    // KRK' = K*RK' // b2
    arm_matrix_instance_f64 KRK_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = (0)
    };
    arm_mat_mult_f64(&K, &RK_transp, &KRK_transp);

    // P_new = EPE' + KRK' // b3
    arm_mat_add_f32(
        (arm_matrix_instance_f32 *)&EPE_transp,
        (arm_matrix_instance_f32 *)&KRK_transp,
        (arm_matrix_instance_f32 *)P
    );

    // TODO current state estimate
    const arm_matrix_instance_f32 *K_const = (arm_matrix_instance_f32 *)&K;
    arm_mat_vec_mult_f32(
        K_const, (float32_t *)&innovation[0], (float32_t *)&state->array[0]
    ); // K * innovation
    arm_add_f64(
        &state->array[0], &state->array[0], &state->array[0], SIZE_STATE
    ); // x + K * innovation

    const quaternion_t state_q = state->attitude; // normalize attitude quaternion
    state->attitude = quaternion_normalize(&state_q);
}

