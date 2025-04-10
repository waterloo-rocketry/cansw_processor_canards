#include "application/estimator/ekf.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "drivers/timer/timer.h"
#include <math.h>
#include <stdint.h>

/*
 * Filter settings --------------------------------
 */
#define SIZE_VECTOR_MAX 13 // set this to the maximum size of any vector here
#define SIZE_ESTIMATOR_STATE 13
#define SIZE_ACCELERATION 3
#define SIZE_IMU_MTI 10
#define SIZE_IMU_ALTIMU 10

#define MS_TO_SECOND 1000.0f

// Weighting, dynamics model
static float Q_diag[SIZE_ESTIMATOR_STATE] = {
    1e-8, 1e-8, 1e-8, 1e-8, 1e0, 1e0, 1e0, 2e-2, 2e-2, 2e-2, 1e-2, 100, 10
};

// Weighting, measurement model: Movella MTI630
static float R_MTI_diag[SIZE_IMU_MTI - SIZE_ACCELERATION] = {
    // Gyro,           Mag,              Baro
    1e-5,
    1e-5,
    1e-5,
    5e-2,
    5e-2,
    5e-2,
    20
};

// Weighting, measurement model: Polulu AltIMU v6
static float R_ALTIMU_diag[SIZE_IMU_ALTIMU - SIZE_ACCELERATION] = {
    // Gyro,           Mag,              Baro
    1e-5,
    1e-5,
    1e-5,
    5e-2,
    5e-2,
    5e-2,
    20
};

/*
 * Helper functions --------------------------------
 * see common/math/math-algebra3d.h
 */

/*
 * Matrix memory space allocations --------------------------------
 */
static float P_data[SIZE_ESTIMATOR_STATE * SIZE_ESTIMATOR_STATE];
static float K_data[SIZE_ESTIMATOR_STATE * SIZE_VECTOR_MAX];
static float buffer1[SIZE_VECTOR_MAX * SIZE_VECTOR_MAX];
static float buffer2[SIZE_VECTOR_MAX * SIZE_VECTOR_MAX];
static float buffer3[SIZE_VECTOR_MAX * SIZE_VECTOR_MAX];
static float buffer4[SIZE_VECTOR_MAX * SIZE_VECTOR_MAX];

static arm_status math_status;

static x_state_t estimator_state __attribute__((unused)) = {0};

static arm_matrix_instance_f32 P;
static const arm_matrix_instance_f32 K;
static arm_matrix_instance_f32 Q;
static arm_matrix_instance_f32 R_MTI;

static float prev_timestamp = 0.0f, current_timestamp = 0.0f, deltaT = 0.0f; // deltaT is in seconds

void ekf_init(void *arg) {
    (void)arg; // unused
    arm_mat_init_f32(&P, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, P_data);

    math_init_matrix_diag(&Q, (uint16_t)SIZE_ESTIMATOR_STATE, Q_diag);

    math_init_matrix_diag(&R_MTI, (uint16_t)SIZE_IMU_MTI, R_MTI_diag);
    // other weighting matrices too...
}

/*
 * Algorithms --------------------------------
 */
void ekf_algorithm(
    x_state_t *state, const u_dynamics_t *input, const y_imu_t *imu_mti, const y_imu_t *bias_mti,
    const y_imu_t *imu_altimu, const y_imu_t *bias_altimu, const float *encoder
) {
    if (W_SUCCESS == timer_get_ms(&current_timestamp)) {
        deltaT = (current_timestamp - prev_timestamp) / MS_TO_SECOND; // convert ms to seconds
        prev_timestamp = current_timestamp;
    }
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
    float state_difference[SIZE_ESTIMATOR_STATE];
    arm_mat_vec_mult_f32(&K, innovation, state_difference);
    arm_add_f32(state, state_difference, state_new.array, SIZE_ESTIMATOR_STATE);
    // state = state_new (but with pointers?)

    // repeat Correct for different sensors: IMU and Encoder
}

// Prediction matric math
// Done, but not checked
void ekf_matrix_predict(
    arm_matrix_instance_f32 *P, const arm_matrix_instance_f32 *F, const arm_matrix_instance_f32 *Q
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
    arm_mat_init_f32(&FT, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer1);
    arm_mat_trans_f32(F, &FT);

    // FP = F*P // b2
    // arm_matrix_instance_f32 FP = {SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer2}; //
    // experiment to "hack" th DSP library,
    // which declares instance and inits it in the same line.
    // should work (as it was not my idea), did not try it though
    arm_matrix_instance_f32 FP;
    arm_mat_init_f32(&FP, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer2);
    arm_mat_mult_f32(F, P, &FP);

    // FPFT = FP * FT // b3
    arm_matrix_instance_f32 FPFT;
    arm_mat_init_f32(&FPFT, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer3);
    arm_mat_mult_f32(&FP, &FT, &FPFT);

    // P_new = FPFT + Q // b1
    arm_matrix_instance_f32 P_new;
    arm_mat_init_f32(&P_new, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer1);
    arm_mat_add_f32(P, Q, &P_new);

    // P = P_new
    arm_mat_init_f32(P, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer1);
}

// Correction matric math
// Done, but not checked
void ekf_matrix_correct(
    arm_matrix_instance_f32 *P, arm_matrix_instance_f32 *K, const arm_matrix_instance_f32 *H,
    const arm_matrix_instance_f32 *R, const uint16_t size_measurement
) {
    // HT = trans(H) // b1
    arm_matrix_instance_f32 HT;
    arm_mat_init_f32(&HT, SIZE_ESTIMATOR_STATE, size_measurement, buffer1);
    arm_mat_trans_f32(H, &HT);

    // PHT = P * HT // b2
    arm_matrix_instance_f32 PHT;
    arm_mat_init_f32(&PHT, SIZE_ESTIMATOR_STATE, size_measurement, buffer2);
    arm_mat_mult_f32(&P, &HT, &PHT);

    // HPHT = H * PHT // b3
    arm_matrix_instance_f32 HPHT;
    arm_mat_init_f32(&HPHT, size_measurement, size_measurement, buffer3);
    arm_mat_mult_f32(&H, &PHT, &HPHT);

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
    arm_mat_init_f32(K, SIZE_ESTIMATOR_STATE, size_measurement, K_data);
    arm_mat_add_f32(&PHT, &Linv, &K);

    // KH = K*H // b3
    arm_matrix_instance_f32 KH;
    arm_mat_init_f32(&KH, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer3);
    arm_mat_mult_f32(&K, &H, &KH);

    // // I = eye // I_data
    arm_matrix_instance_f32 I;
    math_init_matrix_identity(&I, SIZE_ESTIMATOR_STATE);

    // E = I - KH // b2
    arm_matrix_instance_f32 E;
    arm_mat_init_f32(&E, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer2);
    arm_mat_sub_f32(&I, &KH, &E);

    // ET = trans(E) // b1
    arm_matrix_instance_f32 ET;
    arm_mat_init_f32(&ET, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer1);
    arm_mat_trans_f32(&E, &ET);

    // PET = P*ET // b3
    arm_matrix_instance_f32 PET;
    arm_mat_init_f32(&ET, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer3);
    arm_mat_mult_f32(P, &ET, &PET);

    // EPET = E*PET // b1
    arm_matrix_instance_f32 EPET;
    arm_mat_init_f32(&EPET, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer1);
    arm_mat_mult_f32(&E, &PET, &EPET);

    // KT = trans(K) // b2
    arm_matrix_instance_f32 KT;
    arm_mat_init_f32(&KT, size_measurement, SIZE_ESTIMATOR_STATE, buffer2);
    arm_mat_trans_f32(&K, &KT);

    // RKT = R*KT // b3
    arm_matrix_instance_f32 RKT;
    arm_mat_init_f32(&RKT, size_measurement, SIZE_ESTIMATOR_STATE, buffer3);
    arm_mat_mult_f32(&R, &KT, &RKT);

    // KRKT = K*RKT // b2
    arm_matrix_instance_f32 KRKT;
    arm_mat_init_f32(&KRKT, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer2);
    arm_mat_mult_f32(&K, &RKT, &KRKT);

    // P_new = EPET + KRKT // b3
    arm_matrix_instance_f32 P_new;
    arm_mat_init_f32(&P_new, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer3);
    arm_mat_add_f32(&EPET, &KRKT, &P_new);

    // P = P_new
    arm_mat_init_f32(&P, SIZE_ESTIMATOR_STATE, SIZE_ESTIMATOR_STATE, buffer3);
}