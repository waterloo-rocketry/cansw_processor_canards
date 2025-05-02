
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
#define SIZE_VECTOR_MAX 13 // set this to the maximum size of any vector here

/*
 * Matrix memory space allocations --------------------------------
 */

// static const double Q_diag_arr[SIZE_STATE * SIZE_STATE] = {
//     9.15736E-09, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 9.15736E-09, 0, 0, 0, 0, 0,
//     0, 0,           0, 0, 0, 0, 0, 0, 9.15736E-09, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0,
//     0, 0, 9.15736E-09, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 0.915735525, 0, 0, 0,
//     0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 0.915735525, 0, 0, 0, 0, 0, 0, 0,           0, 0,
//     0, 0, 0, 0, 0.915735525, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 0.018314711, 0,
//     0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 0.018314711, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0.018314711, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 0.009157355, 0, 0,
//     0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 91.57355252, 0, 0, 0, 0, 0, 0, 0,           0,
//     0, 0, 0, 0, 0, 9.157355252
// };
// static arm_matrix_instance_f64 Q_dt = {
//     .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = (float64_t *)&Q_diag_arr
// }; // dt* Q

static const double R_MTI_diag_arr[SIZE_IMU_MEAS * SIZE_IMU_MEAS] = {
    0.00001, 0, 0, 0, 0, 0, 0,     0,     0.00001, 0, 0, 0, 0, 0, 0, 0,     0.00001,
    0,       0, 0, 0, 0, 0, 0,     0.005, 0,       0, 0, 0, 0, 0, 0, 0.005, 0,
    0,       0, 0, 0, 0, 0, 0.005, 0,     0,       0, 0, 0, 0, 0, 20
};
static arm_matrix_instance_f64 R_MTI = {
    .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = (float64_t *)&R_MTI_diag_arr
};

// Weighting, measurement model: Polulu AltIMU v6
static const double R_ALTIMU_diag_arr[SIZE_IMU_MEAS * SIZE_IMU_MEAS] = {
    0.00002, 0, 0, 0, 0, 0, 0,     0,     0.00002, 0, 0, 0, 0, 0, 0, 0,     0.00002,
    0,       0, 0, 0, 0, 0, 0,     0.001, 0,       0, 0, 0, 0, 0, 0, 0.001, 0,
    0,       0, 0, 0, 0, 0, 0.001, 0,     0,       0, 0, 0, 0, 0, 30
};
static arm_matrix_instance_f64 R_ALTIMU = {
    .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = (float64_t *)&R_ALTIMU_diag_arr
};

/*
 * Algorithms --------------------------------
 */
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

    u_dynamics_t u_input;
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

        ekf_matrix_correct(x_state, P_flat, &R_MTI, SIZE_STATE, imu_mti, bias_mti);
    }

    if (!is_dead_ALTIMU) {
        // Weighting, measurement model: Polulu AltIMU v6
        double R_ALTIMU_arr[SIZE_IMU_MEAS * SIZE_IMU_MEAS] = {};
        arm_matrix_instance_f64 R_ALTIMU = {
            .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = R_ALTIMU_arr
        };
        const double R_ALTIMU_diag[SIZE_IMU_MEAS] = {2e-5, 2e-5, 2e-5, 1e-3, 1e-3, 1e-3, 3e1};
        math_init_matrix_diag(&R_ALTIMU, (uint16_t)SIZE_IMU_MEAS, R_ALTIMU_diag);

        ekf_matrix_correct(x_state, P_flat, &R_ALTIMU, SIZE_STATE, imu_altimu, bias_altimu);
    }
}

void ekf_matrix_predict(
    x_state_t *x_state, double P_flat[SIZE_STATE * SIZE_STATE], const u_dynamics_t *u_input,
    const double Q_arr[SIZE_STATE * SIZE_STATE], double dt
) {
    double temp_1[SIZE_STATE * SIZE_STATE] = {0};
    double temp_2[SIZE_STATE * SIZE_STATE] = {0};
    double temp_3[SIZE_STATE * SIZE_STATE] = {0};
    double temp_4[SIZE_STATE * SIZE_STATE] = {0};
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
    arm_matrix_instance_f64 F_transp = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = temp_1
    };
    arm_mat_trans_f64(&F, &F_transp);

    // FP = F*P
    arm_matrix_instance_f64 FP = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = temp_2};
    arm_mat_mult_f64(&F, &P, &FP);

    // F * P * F'
    arm_matrix_instance_f64 FPF_transp = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = temp_3
    };
    arm_mat_mult_f64(&FP, &F_transp, &FPF_transp);

    // dt * Q
    arm_matrix_instance_f64 Q = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = Q_arr};
    arm_matrix_instance_f64 Q_dt = {.numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = temp_1};
    arm_mat_scale_f64(&Q, dt, &Q_dt);

    // P_new = FPF' + dt * Q

    arm_mat_add_f64(&FPF_transp, &Q_dt, &P);

    *x_state = state_new;
}

// needs y_imu_t as bias
// corrected state where
void ekf_matrix_correct(
    x_state_t *state, double P_flat[SIZE_STATE * SIZE_STATE], const arm_matrix_instance_f64 *R,
    const uint16_t size_measurement, const y_imu_t *imu, const y_imu_t *bias
) {
    double temp_1[SIZE_STATE * SIZE_STATE] = {0};
    double temp_2[SIZE_STATE * SIZE_STATE] = {0};
    double temp_3[SIZE_STATE * SIZE_STATE] = {0};
    double temp_4[SIZE_STATE * SIZE_STATE] = {0};
    double temp_5[SIZE_STATE * SIZE_STATE] = {0};
    double temp_6[SIZE_STATE * SIZE_STATE] = {0};

    // set up matrix instance for arm operations
    arm_matrix_instance_f64 P = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = (float64_t *)P_flat
    };

    // CORRECTION
    // compute expected measurement and difference to measured values

    const y_imu_t y_expected = model_measurement_imu(state, bias);
    const double y[SIZE_IMU_MEAS]; // imu_1(4:end)
    double innovation[SIZE_IMU_MEAS] = {0};
    arm_sub_f64(
        (float64_t *)&y, (float64_t *)&y_expected.array[0], (float64_t *)&innovation, SIZE_IMU_MEAS
    );

    double h_flat[MEASUREMENT_MODEL_SIZE * SIZE_STATE];
    model_measurement_imu_jacobian(h_flat, state, bias);
    const arm_matrix_instance_f64 H = {
        .numRows = MEASUREMENT_MODEL_SIZE, .numCols = SIZE_STATE, .pData = h_flat
    };

    // compute Kalman gain (and helper matrices)
    // H' = trans(H) // b1
    arm_matrix_instance_f64 H_transp = {
        .numCols = size_measurement, .numRows = SIZE_STATE, .pData = temp_1
    };
    arm_mat_trans_f64(&H, &H_transp);

    // PH' = P * H' // b2
    arm_matrix_instance_f64 PH_transp = {
        .numRows = SIZE_STATE, .numCols = size_measurement, .pData = temp_2
    };
    arm_mat_mult_f64(&P, &H_transp, &PH_transp);

    // HPH' = H * PH' // b3
    arm_matrix_instance_f64 HPH_transp = {
        .numCols = size_measurement, .numRows = size_measurement, .pData = temp_3
    };
    arm_mat_mult_f64(&H, &PH_transp, &HPH_transp);

    // L = HPH' + R // b1
    arm_matrix_instance_f64 L = {
        .numCols = size_measurement, .numRows = size_measurement, .pData = temp_4
    };
    arm_mat_add_f64(&HPH_transp, R, &L);

    // Linv = inv(L) // b3
    arm_matrix_instance_f64 L_inv = {
        .numCols = size_measurement, .numRows = size_measurement, .pData = temp_5
    };
    arm_mat_inverse_f64(&L, &L_inv);

    // Kalman gain
    // K =  PH' * inv(L) // K_data
    arm_matrix_instance_f64 K = {
        .numRows = SIZE_STATE, .numCols = size_measurement, .pData = temp_6
    };
    arm_mat_add_f64(&PH_transp, &L_inv, &K);

    // KH = K*H // b3
    arm_matrix_instance_f64 KH = {
        .numRows = SIZE_STATE, .numCols = size_measurement, .pData = temp_1
    };
    arm_mat_mult_f64(&K, &H, &KH);

    // // I = eye // I_data
    arm_matrix_instance_f64 I;
    math_init_matrix_identity(&I, SIZE_STATE);

    // E = I - KH // b2
    arm_matrix_instance_f64 E = {.numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = temp_2};
    arm_mat_sub_f64(&I, &KH, &E);

    // correct covariance estimate

    // E' = trans(E) // b1
    arm_matrix_instance_f64 E_transp = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = temp_1
    };
    arm_mat_trans_f64(&E, &E_transp);

    // PE' = P*E' // b3
    arm_matrix_instance_f64 PE_transp = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = temp_2
    };
    arm_mat_mult_f64(&P, &E_transp, &PE_transp);

    // EPE' = E*PE' // b1
    arm_matrix_instance_f64 EPE_transp = {
        .numCols = SIZE_STATE, .numRows = SIZE_STATE, .pData = temp_3
    };
    arm_mat_mult_f64(&E, &PE_transp, &EPE_transp);

    // K_transp = trans(K) // b2
    arm_matrix_instance_f64 K_transp = {
        .numCols = SIZE_STATE, .numRows = size_measurement, .pData = temp_4
    };
    arm_mat_trans_f64(&K, &K_transp);

    // RK' = R*K' // b3
    arm_matrix_instance_f64 RK_transp = {
        .numRows = size_measurement, .numCols = SIZE_STATE, .pData = temp_5
    };
    arm_mat_mult_f64(R, &K_transp, &RK_transp);

    // KRK' = K*RK' // b2
    arm_matrix_instance_f64 KRK_transp = {
        .numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = temp_6
    };
    arm_mat_mult_f64(&K, &RK_transp, &KRK_transp);

    // P_new = EPE' + KRK' // b3
    arm_mat_add_f64(&EPE_transp, &KRK_transp, &P);

    // TODO current state estimate
    arm_mat_vec_mult_f64(&K, &innovation[0], &state->array[0]); // K * innovation
    arm_add_f64(
        &state->array[0], &state->array[0], &state->array[0], SIZE_STATE
    ); // x + K * innovation

    const quaternion_t state_q = state->attitude; // normalize attitude quaternion
    state->attitude = quaternion_normalize(&state_q);
}

