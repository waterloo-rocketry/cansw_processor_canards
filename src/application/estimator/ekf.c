#include "src/application/estimator/ekf.h"
#include "common/math/math.h"
#include "arm_math.h"
#include <math.h>

#define SIZE_STATE 13
#define SIZE_ACCELERATION 13
#define SIZE_IMU_MTU 14
#define SIZE_IMU_ALTIMU 10

static const float Q_diag[SIZE_STATE] = {
    1e-10, 1e-10, 1e-10, 1e-10, 1e2, 1e2, 1e2, 2e-2, 2e-2, 2e-2, 1e-2, 1, 10
};

static const float R_MTU_diag[SIZE_IMU_MTU - SIZE_ACCELERATION] = {
    // Gyro,           Mag,              Baro, AHRS
    1e-5, 1e-5, 1e-5,  5e-2, 5e-2, 5e-2,  20,  1e-2, 1e-2, 1e-2, 1e-2
};

arm_status math_status;
arm_matrix_instance_f32 P;
arm_matrix_instance_f32 P_new;

void ekf_algorithm(est_state_t state_new, const est_state_t *state, const est_input_t *input) {
    // Predict
    x_new = model_dynamics_update(state, input, float timestamp, float deltaT); 
    F_x = model_dynamics_jacobian(state, input, float timestamp, float deltaT);
    ekf_predict(&P_new, &P, &F_x, &Q)

    h_x = model_meas_pred(state, bias); 
    H_x = model_meas_jacobian(state, bias);
    ekf_correct(&P_new, &P, &F_x, &Q)
    
}

void ekf_predict(
        arm_matrix_instance_f32 *P_new, 
        const arm_matrix_instance_f32 *P,  
        const arm_matrix_instance_f32 *F_x, 
        const arm_matrix_instance_f32 *Q
    ) {
    arm_matrix_instance_f32 FT;
    arm_mat_trans_f32(F_x, &FT);

    arm_matrix_instance_f32 FP;
    arm_mat_mult_f32(F_x, P, &FP);

    arm_matrix_instance_f32 FPFT;
    arm_mat_mult_f32(&FP, &FT, &FPFT);
    arm_mat_add_f32(P, Q, P_new);
}

void ekf_correct(
    est_state_t *state_new, 
    arm_matrix_instance_f32 *P_new, 
    const est_state_t *state
    const arm_matrix_instance_f32 *P,  
    const arm_matrix_instance_f32 *H_x, 
    const arm_matrix_instance_f32 *R
) {
    arm_matrix_instance_f32 HT;
    // HT = trans(H) // b1
    arm_matrix_instance_f32 PHT;
    // PHT = P * HT // b2, HT free - keep PHT
    arm_matrix_instance_f32 HPHT;
    // HPHT = H * PHT // b1
    arm_matrix_instance_f32 L;
    // L = HPHT + R // b3, HPHT free
    arm_matrix_instance_f32 Linv;
    // Linv = inv(L) // b1, L free

    arm_matrix_instance_f32 K;
    // K =  PHT * Linv // 
    arm_matrix_instance_f32 KH;
    arm_matrix_instance_f32 I;
    arm_matrix_instance_f32 E;
    arm_matrix_instance_f32 ET;
    arm_matrix_instance_f32 PET;
    arm_matrix_instance_f32 EPET;
    arm_matrix_instance_f32 KT;
    arm_matrix_instance_f32 RKT;
    arm_matrix_instance_f32 KRKT;

    /*
    %%% compute Kalman gain (and helper matrices)
    L = H * P * H' + R;
    K = P * H' * inv(L);
    E = eye(length(x)) - K * H;
    
    %%% correct covariance estimate
    P_corr = E * P * E' + K * R * K'; % joseph stabilized
    */
}

void math_matrix_identity(arm_matrix_instance_f32 *I, const float size) {
    float I_data[size * size];    
    for (uint16_t i = 0; i < size; i++) {
        for (uint16_t j = 0; j < size; j++) {
            I_data[i * size + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    arm_mat_init_f32(I, size, size, I_data);
}