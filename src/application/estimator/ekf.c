#include "src/application/estimator/ekf.h"
#include "common/math/math.h"
#include "arm_math.h"
#include <math.h>
#include <stdint.h>

/*
* Filter settings --------------------------------
*/
#define SIZE_MAX 13 // set this to the maximum size of any vector here
#define SIZE_STATE 13
#define SIZE_ACCELERATION 3
#define SIZE_IMU_MTI 10
#define SIZE_IMU_ALTIMU 10

// Weighting, dynamics model 
static const float Q_diag[SIZE_STATE] = {
    1e-10, 1e-10, 1e-10, 1e-10, 1e2, 1e2, 1e2, 2e-2, 2e-2, 2e-2, 1e-2, 1, 10
};

// Weighting, measurement model: Movella MTI630
static const float R_MTI_diag[SIZE_IMU_MTU - SIZE_ACCELERATION] = {
    // Gyro,           Mag,              Baro
    1e-5, 1e-5, 1e-5,  5e-2, 5e-2, 5e-2,  20
};

// Weighting, measurement model: Polulu AltIMU v6
static const float R_ALTIMU_diag[SIZE_IMU_MTU - SIZE_ACCELERATION] = {
    // Gyro,           Mag,              Baro
    1e-5, 1e-5, 1e-5,  5e-2, 5e-2, 5e-2,  20
};


/*
* Helper functions --------------------------------
* can we put these somewhere else to clean up this file?
*/
// creates matrix instance, matrix is identity of chosen size
void math_init_matrix_identity(arm_matrix_instance_f32 *I, const uint16_t size) {
    float I_data[size * size];    
    for (uint16_t i = 0; i < size; i++) {
        for (uint16_t j = 0; j < size; j++) {
            I_data[i * size + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    arm_mat_init_f32(I, size, size, &I_data);
}

// creates matrix instance, matrix is diagonal matrix filled with entries of a 1D float array (vector). Zeros elsewhere. 
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
static float P_data  [SIZE_STATE * SIZE_STATE];
static float K_data  [SIZE_STATE * SIZE_MAX];
static float buffer1 [SIZE_MAX * SIZE_MAX];
static float buffer2 [SIZE_MAX * SIZE_MAX];
static float buffer3 [SIZE_MAX * SIZE_MAX];
static float buffer4 [SIZE_MAX * SIZE_MAX];

arm_status math_status;

static arm_matrix_instance_f32 P;
arm_mat_init_f32(&P, SIZE_STATE, SIZE_STATE, P_data);

arm_matrix_instance_f32 K;

static arm_matrix_instance_f32 Q;
math_init_matrix_diag(&Q, SIZE_STATE, Q_diag);
static arm_matrix_instance_f32 R_MTI;
math_init_matrix_diag(&R_MTI, SIZE_IMU_MTI, R_MTI_diag);
// other weighting matrices too...

/*
* Algorithms --------------------------------
*/
void ekf_algorithm(
    x_state_t *state, 
    const uint32_t timestamp,
    const uint32_t timestamp_new, // how do we do this? need time difference to previous execution. Dynamics model needs it as float
    const u_dynamics_t *input, 
    const y_imu_t *imu_mti,
    const y_imu_t *bias_mti,
    const y_imu_t *imu_altimu,
    const y_imu_t *bias_altimu,
    const float *encoder
) {
    // add acceleration model to calculate input.acceleration, as input only has cmd right now

    // Predict
    arm_matrix_instance_f32 F_x = model_dynamics_jacobian(state, input, timestamp, deltaT);
    x_state_t state_new = model_dynamics_update(state, input, timestamp, deltaT); 
    ekf_matrix_predict(&P, &F_x, &Q);
    // state = state_new (but with pointers?)

    // Correct (for one IMU)
    // add if(not dead)
    arm_matrix_instance_f32 H_x = model_meas_mti_jacobian(state, bias_mti);
    ekf_matrix_correct(&P, &K, &H_x, &R_MTI, SIZE_IMU_MTI);
    y_imu_t h_x = model_meas_mti_pred(state, bias_mti);
    float innovation[SIZE_IMU_MTI];
    arm_sub_f32(imu_mti->array, h_x.array, innovation, SIZE_IMU_MTI);
    float state_difference[SIZE_STATE];
    arm_mat_vec_mult_f32(&K, innovation, state_difference);
    arm_add_f32(state, state_difference, state_new.array, SIZE_STATE);
    // state = state_new (but with pointers?)

    // repeat Correct for different sensors: IMU and Encoder
}

// Prediction matric math 
// Done, but not checked
void ekf_matrix_predict(
        arm_matrix_instance_f32 *P,  
        const arm_matrix_instance_f32 *F, 
        const arm_matrix_instance_f32 *Q
    ) {
    
    /*
    // maybe we can define macros, not shorten code massively, and improve readability. 
    // Do this for differen arm_mat_i functions: slightly differing definitions, eg multiple inputs
    // advantage: declares and initializes output, much shorter 
    #define trans(OUT, IN, MEM)\ 
    arm_matrix_instance_f32 OUT; \
    arm_mat_init_f32(&OUT, IN.numRows, IN.numCols, MEM); \
    arm_mat_trans_f32(IN, &OUT);
    */

    // FT = trans(F) // b1
    // trans(&FT, &F, buffer1) // see above for this alternative
    arm_matrix_instance_f32 FT;
    arm_mat_init_f32(&FT, SIZE_STATE, SIZE_STATE, buffer1);
    arm_mat_trans_f32(F, &FT);

    // FP = F*P // b2
    //arm_matrix_instance_f32 FP = {SIZE_STATE, SIZE_STATE, buffer2}; // experiment to "hack" th DSP library, 
    // which declares instance and inits it in the same line. 
    // should work (as it was not my idea), did not try it though
    arm_matrix_instance_f32 FP;
    arm_mat_init_f32(&FP, SIZE_STATE, SIZE_STATE, buffer2);
    arm_mat_mult_f32(F, P, &FP);

    // FPFT = FP * FT // b3
    arm_matrix_instance_f32 FPFT;
    arm_mat_init_f32(&FPFT, SIZE_STATE, SIZE_STATE, buffer3);
    arm_mat_mult_f32(&FP, &FT, &FPFT);

    // P_new = FPFT + Q // b1
    arm_matrix_instance_f32 P_new;
    arm_mat_init_f32(&P_new, SIZE_STATE, SIZE_STATE, buffer1);
    arm_mat_add_f32(P, Q, &P_new);

    // P = P_new
    arm_mat_init_f32(P, SIZE_STATE, SIZE_STATE, buffer1);
}

// Correction matric math 
// Done, but not checked
void ekf_matrix_correct(
    arm_matrix_instance_f32 *P,  
    arm_matrix_instance_f32 *K,
    const arm_matrix_instance_f32 *H, 
    const arm_matrix_instance_f32 *R,
    const uint16_t size_measurement 
) {
    // HT = trans(H) // b1
    arm_matrix_instance_f32 HT;
    arm_mat_init_f32(&HT, SIZE_STATE, size_measurement, buffer1);
    arm_mat_trans_f32(H, &HT);

    // PHT = P * HT // b2
    arm_matrix_instance_f32 PHT;
    arm_mat_init_f32(&PHT, SIZE_STATE, size_measurement, buffer2);
    arm_mat_mult_f32(P, &HT, &PHT);

    // HPHT = H * PHT // b3
    arm_matrix_instance_f32 HPHT;
    arm_mat_init_f32(&HPHT, size_measurement, size_measurement, buffer3);
    arm_mat_mult_f32(H, &PHT, &HPHT);

    // L = HPHT + R // b1
    arm_matrix_instance_f32 L;
    arm_mat_init_f32(&L, size_measurement, size_measurement, buffer1);
    arm_mat_add_f32(&HPHT, R, &L);

    // Linv = inv(L) // b3
    arm_matrix_instance_f32 Linv;
    arm_mat_init_f32(&L, size_measurement, size_measurement, buffer3);
    math_status = arm_mat_inverse_f32(&L, &Linv);	
    
    // Kalman gain 
    // K =  PHT * Linv // K_data
    arm_mat_init_f32(K, SIZE_STATE, size_measurement, K_data);
    arm_mat_add_f32(&PHT, &Linv, K);

    // KH = K*H // b3
    arm_matrix_instance_f32 KH;
    arm_mat_init_f32(&KH, SIZE_STATE, SIZE_STATE, buffer3);
    arm_mat_mult_f32(K, &H, &KH);

    // // I = eye // I_data
    arm_matrix_instance_f32 I;
    math_init_matrix_identity(&I, SIZE_STATE);

    // E = I - KH // b2
    arm_matrix_instance_f32 E;
    arm_mat_init_f32(&E, SIZE_STATE, SIZE_STATE, buffer2);
    arm_mat_sub_f32(&I, &KH, &E) 	

    // ET = trans(E) // b1
    arm_matrix_instance_f32 ET;
    arm_mat_init_f32(&ET, SIZE_STATE, SIZE_STATE, buffer1);
    arm_mat_trans_f32(&E, &ET);
    
    // PET = P*ET // b3
    arm_matrix_instance_f32 PET;
    arm_mat_init_f32(&ET, SIZE_STATE, SIZE_STATE, buffer3);
    arm_mat_mult_f32(P, &ET, &PET);
    
    // EPET = E*PET // b1
    arm_matrix_instance_f32 EPET;
    arm_mat_init_f32(&EPET, SIZE_STATE, SIZE_STATE, buffer1);
    arm_mat_mult_f32(&E, &PET, &EPET);

    // KT = trans(K) // b2
    arm_matrix_instance_f32 KT;
    arm_mat_init_f32(&KT, size_measurement, SIZE_STATE, buffer2);
    arm_mat_trans_f32(K, &KT);
     
    // RKT = R*KT // b3
    arm_matrix_instance_f32 RKT;
    arm_mat_init_f32(&RKT, size_measurement, SIZE_STATE, buffer3);
    arm_mat_mult_f32(&E, &PET, &EPET);
    
    // KRKT = K*RKT // b2
    arm_matrix_instance_f32 KRKT;
    arm_mat_init_f32(&KRKT, SIZE_STATE, SIZE_STATE, buffer2);
    arm_mat_mult_f32(K, &KRT, &KRKT);

    // P_new = EPET + KRKT // b3
    arm_matrix_instance_f32 P_new;
    arm_mat_init_f32(&P_new, SIZE_STATE, SIZE_STATE, buffer3);
    arm_mat_add_f32(&EPET, &KRKT, &P_new);

    // P = P_new
    arm_mat_init_f32(P, SIZE_STATE, SIZE_STATE, buffer3);        
}