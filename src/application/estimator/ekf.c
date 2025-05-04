
#include "application/estimator/ekf.h"
#include "application/estimator/model/model_acceleration.h"
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

static double temp_1[SIZE_STATE * SIZE_STATE] = {0};
static double temp_2[SIZE_STATE * SIZE_STATE] = {0};
static double temp_3[SIZE_STATE * SIZE_STATE] = {0};
static double temp_4[SIZE_STATE * SIZE_STATE] = {0};

static inline void reset_temp_matrix(double *matrix) {
    memset(matrix, 0, SIZE_STATE * SIZE_STATE * sizeof(double));
}

/*
 * Algorithms --------------------------------
 */

void ekf_matrix_predict(
    x_state_t *x_state, double P_flat[SIZE_STATE * SIZE_STATE], const u_dynamics_t *u_input,
    double Q_arr[SIZE_STATE * SIZE_STATE], double dt
) {
    x_state_t state_new = {0};

    // set up matrix instance for arm operations
    arm_matrix_instance_f64 P = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = (float64_t *)P_flat
    };

    // DISCRETE DYANMICS UPDATE
    state_new = model_dynamics_update(x_state, u_input, dt);

    // discrete jacobian: F = df/dx
    double F_flat[SIZE_STATE * SIZE_STATE] = {0};
    model_dynamics_jacobian(F_flat, x_state, u_input, dt);
    arm_matrix_instance_f64 F = {.numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = F_flat};

    // DISCRETE COVARIANCE
    // F'
    reset_temp_matrix(temp_1);
    arm_matrix_instance_f64 F_transp = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = temp_1
    };
    arm_mat_trans_f64(&F, &F_transp);

    // FP = F*P
    reset_temp_matrix(temp_2);
    arm_matrix_instance_f64 FP = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = temp_2};
    arm_mat_mult_f64(&F, &P, &FP);

    // F * P * F'
    reset_temp_matrix(temp_3);
    arm_matrix_instance_f64 FPF_transp = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = temp_3
    };
    arm_mat_mult_f64(&FP, &F_transp, &FPF_transp);

    // dt * Q
    reset_temp_matrix(temp_4);
    arm_matrix_instance_f64 Q = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = Q_arr};
    arm_matrix_instance_f64 Q_dt = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = temp_4};
    arm_mat_scale_f64(&Q, dt, &Q_dt);

    // P_new = FPF' + dt * Q

    arm_mat_add_f64(&FPF_transp, &Q_dt, &P);

    *x_state = state_new;
}

void ekf_matrix_correct(
    x_state_t *x_state, double P_flat[SIZE_STATE * SIZE_STATE], const arm_matrix_instance_f64 *R,
    double y_meas[SIZE_IMU_MEAS], const double *bias
) {
    // set up matrix instance for arm operations
    arm_matrix_instance_f64 P = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = (float64_t *)P_flat
    };

    // CORRECTION
    // compute expected measurement and difference to measured values

    // y = IMU_1(4:end)

    const y_imu_t y_expected_full = model_measurement_imu(x_state, bias);
    double y_expected[SIZE_IMU_MEAS] = {0};
    memcpy(y_expected, &y_expected_full.array[3], SIZE_IMU_MEAS * sizeof(double));

    double innovation[SIZE_IMU_MEAS] = {0};
    arm_sub_f64(y_meas, y_expected, innovation, SIZE_IMU_MEAS);

    double h_flat[SIZE_IMU_MEAS * SIZE_STATE] = {0};
    model_measurement_imu_jacobian(h_flat, x_state, (y_imu_t *)bias);
    const arm_matrix_instance_f64 H = {
        .numRows = SIZE_IMU_MEAS, .numCols = SIZE_STATE, .pData = h_flat
    };

    // compute Kalman gain (and helper matrices)
    // H' = trans(H) // b1
    reset_temp_matrix(temp_1);
    arm_matrix_instance_f64 H_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_IMU_MEAS, .pData = temp_1
    };
    arm_mat_trans_f64(&H, &H_transp);

    // PH' = P * H' // b2
    reset_temp_matrix(temp_2);
    arm_matrix_instance_f64 PH_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_IMU_MEAS, .pData = temp_2
    };
    arm_mat_mult_f64(&P, &H_transp, &PH_transp);

    // HPH' = H * PH' // b3
    reset_temp_matrix(temp_1);
    arm_matrix_instance_f64 HPH_transp = {
        .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = temp_1
    };
    arm_mat_mult_f64(&H, &PH_transp, &HPH_transp);

    // L = HPH' + R // b1
    reset_temp_matrix(temp_3);
    arm_matrix_instance_f64 L = {
        .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = temp_3
    };
    arm_mat_add_f64(&HPH_transp, R, &L);

    // Linv = inv(L) // b3
    reset_temp_matrix(temp_1);
    arm_matrix_instance_f64 L_inv = {
        .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = temp_1
    };
    arm_mat_inverse_f64(&L, &L_inv);

    // Kalman gain
    // K =  PH' * inv(L) // K_data
    reset_temp_matrix(temp_3);
    arm_matrix_instance_f64 K = {.numRows = SIZE_STATE, .numCols = SIZE_IMU_MEAS, .pData = temp_3};
    arm_mat_mult_f64(&PH_transp, &L_inv, &K);

    // KH = K*H // b3
    reset_temp_matrix(temp_1);
    arm_matrix_instance_f64 KH = {.numRows = SIZE_STATE, .numCols = SIZE_IMU_MEAS, .pData = temp_1};
    arm_mat_mult_f64(&K, &H, &KH);

    // // I = eye // I_data
    reset_temp_matrix(temp_2);
    arm_matrix_instance_f64 I = {.numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = temp_2};
    math_init_matrix_identity(&I, SIZE_STATE);

    // E = I - KH // b2
    reset_temp_matrix(temp_4);
    arm_matrix_instance_f64 E = {.numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = temp_4};
    arm_mat_sub_f64(&I, &KH, &E);

    // correct covariance estimate

    // E' = trans(E) // b1
    reset_temp_matrix(temp_1);
    arm_matrix_instance_f64 E_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = temp_1
    };
    arm_mat_trans_f64(&E, &E_transp);

    // PE' = P*E' // b3
    reset_temp_matrix(temp_2);
    arm_matrix_instance_f64 PE_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = temp_2
    };
    arm_mat_mult_f64(&P, &E_transp, &PE_transp);

    // EPE' = E*PE' // b1
    reset_temp_matrix(temp_1);
    arm_matrix_instance_f64 EPE_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = temp_1
    };
    arm_mat_mult_f64(&E, &PE_transp, &EPE_transp);

    // K_transp = trans(K) // b2
    reset_temp_matrix(temp_2);
    arm_matrix_instance_f64 K_transp = {
        .numRows = SIZE_IMU_MEAS, .numCols = SIZE_STATE, .pData = temp_2
    };
    arm_mat_trans_f64(&K, &K_transp);

    // RK' = R*K' // b3
    reset_temp_matrix(temp_4);
    arm_matrix_instance_f64 RK_transp = {
        .numCols = SIZE_STATE, .numRows = SIZE_IMU_MEAS, .pData = temp_4
    };
    arm_mat_mult_f64(R, &K_transp, &RK_transp);

    // KRK' = K*RK' // b2
    reset_temp_matrix(temp_2);
    arm_matrix_instance_f64 KRK_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = temp_2
    };
    arm_mat_mult_f64(&K, &RK_transp, &KRK_transp);

    // P_new = EPE' + KRK' // b3
    arm_mat_add_f64(&EPE_transp, &KRK_transp, &P);

    // K * innovation
    x_state_t k_innovation = {0};
    arm_mat_vec_mult_f64(&K, &innovation[0], k_innovation.array);
    // x_corr = x + K * innovation;
    x_state_t x_corr = {0};
    arm_add_f64(k_innovation.array, x_state->array, x_corr.array, SIZE_STATE);

    for (int i = 0; i < SIZE_STATE; i++) {
        x_state->array[i] = x_corr.array[i];
    }
    const quaternion_t state_q = x_state->attitude; // normalize attitude quaternion
    x_state->attitude = quaternion_normalize(&state_q);
}

void ekf_algorithm(
    x_state_t *x_state, double P_flat[SIZE_STATE * SIZE_STATE], const y_imu_t *imu_mti,
    const y_imu_t *bias_mti, const y_imu_t *imu_altimu, const y_imu_t *bias_altimu, double cmd,
    double encoder, double dt, const bool is_dead_MTI, const bool is_dead_ALTIMU
) {
    // Prediction step
    // %%% Q is a square 13 matrix, tuning for prediction E(noise)
    // %%% x = [   q(4),           w(3),           v(3),      alt(1), Cl(1), delta(1)]
    double Q_diag[SIZE_STATE] = {
        1e-8, 1e-8, 1e-8, 1e-8, 1e0, 1e0, 1e0, 2e-2, 2e-2, 2e-2, 1e-2, 100, 10
    };
    double Q_arr[SIZE_STATE * SIZE_STATE] = {0};
    arm_matrix_instance_f64 Q = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = Q_arr};
    math_init_matrix_diag(&Q, (uint16_t)SIZE_STATE, Q_diag);

    u_dynamics_t u_input = {0};
    u_input.acceleration =
        model_acceleration(x_state, imu_mti, is_dead_MTI, imu_altimu, is_dead_ALTIMU);
    u_input.cmd = cmd;

    // Predict
    ekf_matrix_predict(x_state, P_flat, &u_input, Q.pData, dt);

    // %% Correction step(s), sequential for each IMU
    // %%% R is a square matrix (size depending on amount of sensors), tuning for measurement
    // E(noise)

    // todo: encoder revival

    // only correct with alive IMUs
    if (!is_dead_MTI) {
        // // Weighting, measurement model: MTi630
        static double R_MTI_arr[SIZE_IMU_MEAS * SIZE_IMU_MEAS] = {};
        static arm_matrix_instance_f64 R_MTI = {
            .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = R_MTI_arr
        };
        const double R_MTI_diag[SIZE_IMU_MEAS] = {1e-5, 1e-5, 1e-5, 5e-3, 5e-3, 5e-3, 2e1};
        math_init_matrix_diag(&R_MTI, (uint16_t)SIZE_IMU_MEAS, R_MTI_diag);

        double imu_mti_arr[SIZE_IMU_MEAS] = {0};
        memcpy(imu_mti_arr, &imu_mti->array[3], SIZE_IMU_MEAS * sizeof(double));
        ekf_matrix_correct(x_state, P_flat, &R_MTI, imu_mti_arr, bias_mti);
    }

    if (!is_dead_ALTIMU) {
        // Weighting, measurement model: Polulu AltIMU v6
        double R_ALTIMU_arr[SIZE_IMU_MEAS * SIZE_IMU_MEAS] = {};
        arm_matrix_instance_f64 R_ALTIMU = {
            .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = R_ALTIMU_arr
        };
        const double R_ALTIMU_diag[SIZE_IMU_MEAS] = {2e-5, 2e-5, 2e-5, 1e-3, 1e-3, 1e-3, 3e1};
        math_init_matrix_diag(&R_ALTIMU, (uint16_t)SIZE_IMU_MEAS, R_ALTIMU_diag);
        double imu_altimu_arr[SIZE_IMU_MEAS] = {0};
        memcpy(imu_altimu_arr, &imu_altimu->array[3], SIZE_IMU_MEAS * sizeof(double));
        ekf_matrix_correct(x_state, P_flat, &R_ALTIMU, imu_altimu_arr, bias_altimu);
    }
}

