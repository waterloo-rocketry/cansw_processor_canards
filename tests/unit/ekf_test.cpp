#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "application/estimator/ekf.h"
#include "application/estimator/estimator_types.h"
#include "arm_math.h"
#include "common/math/math-algebra3d.h"

DEFINE_FFF_GLOBALS;
}

class EstimatorEKFTest : public ::testing::Test {
protected:
    void SetUp() override {
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

/**
 *
SIZE_STATE = 13;

% Initial state
x = [
    0.814723686393179;
    0.905791937075619;
    0.126986816293506;
    0.913375856139019;
    0.632359246225410;
    0.097540404999410;
    0.278498218867048;
    0.546881519204984;
    0.957506835434298;
    0.964888535199277;
    0.157613081677548;
    0.970592781760616;
    0.957166948242946
];

% Initial covariance matrix
P = zeros(SIZE_STATE);

% Control input as struct
u = struct();
u.cmd = 4.217612826262750;
u.accel = [1.456126946168524; 2.400841406666400; 0.425659015881646];

% Time step
dt = 0.915735525189067;

% Process noise covariance
Q = diag([ones(1,4)*1e-8, ones(1,3)*1e0, ones(1,3)*2e-2, 1e-2, 100, 10]);

% Call EKF prediction
[xhat, Phat] = ekf_predict(@model_dynamics, @model_dynamics_jacobian, x, P, u, Q, dt);

% Display state vector
disp('Predicted state x:');
disp(xhat);

% Display flattened P in C array format
fprintf('\nFlattened Phat (C-style row-major):\n');
Phat_flat = Phat';              % transpose to row-major order
Phat_flat = Phat_flat(:);       % linearize

fprintf('double P_flat[%d] = {\n', numel(Phat_flat));
for i = 1:numel(Phat_flat)
    if i < numel(Phat_flat)
        fprintf('    %.15g,\n', Phat_flat(i));
    else
        fprintf('    %.15g\n', Phat_flat(i));
    end
end
fprintf('};\n');


 */
TEST_F(EstimatorEKFTest, EKFPredictNominalCheck) {
    // Arrange
    x_state_t state = {
        .array = {
            0.814723686393179,
            0.905791937075619,
            0.126986816293506,
            0.913375856139019,
            0.632359246225410,
            0.097540404999410,
            0.278498218867048,
            0.546881519204984,
            0.957506835434298,
            0.964888535199277,
            0.157613081677548,
            0.970592781760616,
            0.957166948242946
        }
    };
    double P_flat[SIZE_STATE * SIZE_STATE] = {0};
    const u_dynamics_t input = {
        .cmd = 4.217612826262750,
        .acceleration = {.array = {1.456126946168524, 2.400841406666400, 0.425659015881646}}
    };
    double dt = 0.915735525189067;

    // set up Q like ekf_algorithm would
    double Q_diag[SIZE_STATE] = {
        1e-8, 1e-8, 1e-8, 1e-8, 1e0, 1e0, 1e0, 2e-2, 2e-2, 2e-2, 1e-2, 100, 10
    };
    double Q_arr[SIZE_STATE * SIZE_STATE] = {0};
    arm_matrix_instance_f64 Q = {.numRows = SIZE_STATE, .numCols = SIZE_STATE, .pData = Q_arr};
    math_init_matrix_diag(&Q, (uint16_t)SIZE_STATE, Q_diag);

    x_state_t expected_state = {
        .array = {
            0.268274165208879,
            0.696419511023482,
            0.194712315844409,
            0.636487196262266,
            0.643134076016596,
            0.006279518794539,
            0.471014707988784,
            -0.400684442488099,
            8.416935880119659,
            -6.314095921040007,
            0.525564693616072,
            1.457215728968408,
            60.671289317422811
        }
    };
    // double expected_P_flat[SIZE_STATE * SIZE_STATE] = {
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
    double expected_P_flat[169] = {9.15735525189067e-09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   9.15735525189067e-09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   9.15735525189067e-09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   9.15735525189067e-09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0.915735525189067,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0.915735525189067,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0.915735525189067,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0.0183147105037813,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0.0183147105037813,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0.0183147105037813,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0.00915735525189067,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   91.5735525189067,     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   9.15735525189067};

    // Act
    ekf_matrix_predict(&state, P_flat, &input, Q.pData, dt);

    // Assert
    double tolerance = 1e-6;

    // check x
    for (int i = 0; i < SIZE_STATE; i++) {
        EXPECT_NEAR(state.array[i], expected_state.array[i], tolerance);
    }

    // check P
    for (int i = 0; i < SIZE_STATE * SIZE_STATE; i++) {
        EXPECT_NEAR(P_flat[i], expected_P_flat[i], tolerance);
    }
}