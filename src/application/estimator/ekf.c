#include "src/application/estimator/ekf.h"
#include "common/math/math.h"
#include "arm_math.h"
#include <math.h>

/*
* Filter settings --------------------------------
*/
#define SIZE_MAX 13 // set this to the maximum size of any vector here
#define SIZE_STATE 13
#define SIZE_ACCELERATION 3
#define SIZE_IMU_MTU 10
#define SIZE_IMU_ALTIMU 10

static const float Q_diag[SIZE_STATE] = {
    1e-10, 1e-10, 1e-10, 1e-10, 1e2, 1e2, 1e2, 2e-2, 2e-2, 2e-2, 1e-2, 1, 10
};

static const float R_MTU_diag[SIZE_IMU_MTU - SIZE_ACCELERATION] = {
    // Gyro,           Mag,              Baro
    1e-5, 1e-5, 1e-5,  5e-2, 5e-2, 5e-2,  20
};


/*
* Helper functions --------------------------------
*/
void math_init_matrix_identity(arm_matrix_instance_f32 *I, const uint16_t size) {
    float I_data[size * size];    
    for (uint16_t i = 0; i < size; i++) {
        for (uint16_t j = 0; j < size; j++) {
            I_data[i * size + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    arm_mat_init_f32(I, size, size, &I_data);
}

void math_init_matrix_diag(arm_matrix_instance_f32 *matrix, const uint16_t size, const float *vector) {
    float matrix_data[size * size];    
    for (uint16_t i = 0; i < size; i++) {
        for (uint16_t j = 0; j < size; j++) {
            matrix_data[i * size + j] = (i == j) ? *vector[i] : 0.0f;
        }
    }
    arm_mat_init_f32(matrix, size, size, &matrix_data);
}

/*
* Matrix memory space allocations --------------------------------
*/
float P_data  [SIZE_STATE * SIZE_STATE];
float K_data  [SIZE_STATE * SIZE_MAX];
float buffer1 [SIZE_MAX * SIZE_MAX];
float buffer2 [SIZE_MAX * SIZE_MAX];
float buffer3 [SIZE_MAX * SIZE_MAX];
float buffer4 [SIZE_MAX * SIZE_MAX];

arm_status math_status;

static arm_matrix_instance_f32 P;
arm_mat_init_f32(&P, SIZE_STATE, SIZE_STATE, float *P_data);

arm_matrix_instance_f32 K;

arm_matrix_instance_f32 Q;
math_init_matrix_diag(&Q, SIZE_STATE, &Q_diag);
arm_matrix_instance_f32 R_MTU;
math_init_matrix_diag(&R_MTU, SIZE_IMU_MTU, &R_MTU_diag);

/*
* Algorithms --------------------------------
*/
void ekf_algorithm(
    est_state_t *state, 
    const est_input_t *input, 
    const estimator_measurement_imu_t *IMU_MTU,
    const estimator_measurement_imu_t *IMU_ALTIMU,
    const estimator_measurement_encoder_t *encoder
) {
    // Predict
    float F_x = model_dynamics_jacobian(&state, &input, float timestamp, float deltaT);
    float state_new = model_dynamics_update(&state, &input, float timestamp, float deltaT); 
    ekf_matrix_predict(&P, &F_x, &Q);

    // Correct (for one IMU)
    float H_x = model_meas_mtu_jacobian(&state, &bias);
    ekf_matrix_correct(&P, &K, &H_x, &R_MTU, size_measurement);
    float h_x = model_meas_mtu_pred(&state, &bias); 
    float innovation[SIZE_IMU_MTU];
    arm_sub_f32(IMU_MTU->sensor.array, &h_x, &innovation, SIZE_IMU_MTU);
    float state_difference[SIZE_STATE];
    arm_mat_vec_mult_f32(&K, &innovation, &state_difference);
    arm_add_f32(&state, &state_difference, &state_new, SIZE_STATE);

}

void ekf_matrix_predict(
        arm_matrix_instance_f32 *P,  
        const arm_matrix_instance_f32 *F_x, 
        const arm_matrix_instance_f32 *Q
    ) {
    arm_matrix_instance_f32 FT;
    // FT = trans(F) // b1
    arm_mat_trans_f32(F_x, &FT);
    
    arm_matrix_instance_f32 FP;
    // FP = F*P // b2
    arm_mat_mult_f32(F_x, P, &FP);

    arm_matrix_instance_f32 FPFT;
    // FPFT = FP * FT // FP free
    arm_mat_mult_f32(&FP, &FT, &FPFT);
    // P_new = FPFT + Q 
    arm_mat_add_f32(P, Q, P_new);
}

void ekf_matrix_correct(
    arm_matrix_instance_f32 *P,  
    arm_matrix_instance_f32 *K,
    const arm_matrix_instance_f32 *H_x, 
    const arm_matrix_instance_f32 *R,
    const uint16_t size_measurement
) {
    arm_matrix_instance_f32 HT;
    // HT = trans(H) // b1
    arm_matrix_instance_f32 PHT;
    // PHT = P * HT // b2, HT free - keep PHT in b2
    arm_matrix_instance_f32 HPHT;
    // HPHT = H * PHT // b1
    arm_matrix_instance_f32 L;
    // L = HPHT + R // b3, HPHT free
    arm_matrix_instance_f32 Linv;
    // Linv = inv(L) // b1, L free

    arm_mat_init_f32(K, SIZE_STATE, size_measurement, float *K_data);
    // K =  PHT * Linv // b3, Linv free, PHT free
    arm_matrix_instance_f32 KH;
    // KH = K*H // b2
    arm_matrix_instance_f32 I;
    // I = eye // b1 
    arm_matrix_instance_f32 E;
    // E = I - KH // b4, KH free, I free
    arm_matrix_instance_f32 ET;
    // ET = trans(E) // b1
    arm_matrix_instance_f32 PET;
    // PET = P*ET // b2, ET free
    arm_matrix_instance_f32 EPET;
    // EPET = E*PET // b1, PET free, E free

    arm_matrix_instance_f32 KT;
    // KT = trans(K) // b2 
    arm_matrix_instance_f32 RKT;
    // RKT = R*KT // b4, KT fee
    arm_matrix_instance_f32 KRKT;
    // KRKT = K*RKT // b2

    /*
    %%% compute Kalman gain (and helper matrices)
    L = H * P * H' + R;
    K = P * H' * inv(L);
    E = eye(length(x)) - K * H;
    
    %%% correct covariance estimate
    P_corr = E * P * E' + K * R * K'; % joseph stabilized
    */
}

