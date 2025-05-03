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

// clang-format off
/**
 *
SIZE_STATE = 13;
SIZE_MEAS = 7; % IMU measurement (from element 4 to end: 3 gyroscope + 3 magnetometer + 1 barometer)

% Initial state estimate
x = [
    0.268274165208879;
    0.696419511023482;
    0.194712315844409;
    0.636487196262266;
    0.643134076016596;
    0.006279518794539;
    0.471014707988784;
    -0.400684442488099;
    8.416935880119659;
    -6.314095921040007;
    0.525564693616072;
    1.457215728968408;
    60.671289317422811
];

% Initial covariance (some nonzero values for realism)
P = eye(SIZE_STATE) * 0.1;

% Example of IMU_1 data (a 10-item array)
% IMU_1 = [accel(1), accel(2), accel(3), gyro(1), gyro(2), gyro(3), mag(1), mag(2), mag(3), barometer]
IMU_1 = [
    0.01;   % accelerometer X
    0.02;   % accelerometer Y
    -0.01;  % accelerometer Z
    0.3;    % gyroscope X
    -0.2;   % gyroscope Y
    0.5;    % gyroscope Z
    0.2;    % magnetometer X
    -0.1;   % magnetometer Y
    0.4;    % magnetometer Z
    1013.25 % barometer
];

% Measurement vector y (IMU_1(4:end): Gyroscope, Magnetometer, Barometer)
y = IMU_1(4:end);  % Select elements 4 to 10 from IMU_1

% Bias vector b (example for 10 elements)
b = struct();
b.bias_1 = [
    0.001; -0.002; 0.003;   % Accelerometer bias (X, Y, Z)
    0.0001; -0.0002; 0.0003; % Gyroscope bias (X, Y, Z)
    0.001; -0.002; 0.003;   % Magnetometer bias (X, Y, Z)
    0.005                 % Barometer bias
];

% Measurement noise covariance
R = diag([ones(1,3)*1e-5, ones(1,3)*5e-3, ones(1,1)*2e1]);

% Run correction step
[xhat, Phat] = ekf_correct(@model_meas_imu, @model_meas_imu_jacobian, x, P, y, b.bias_1, R);

% Output results
disp('Corrected state xhat:');
disp(xhat);

% Print the flattened covariance matrix P in C-style
fprintf('\nFlattened Phat (C-style row-major):\n');
Phat_flat = Phat'; % convert to row-major
Phat_flat = Phat_flat(:);
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
// clang-format on
TEST_F(EstimatorEKFTest, EKFCorrectNominalCheck) {
    // Arrange
    // Initial state estimate
    x_state_t state = {
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

    // Covariance matrix

    double P_flat[SIZE_STATE * SIZE_STATE] = {0};

    // % Initial covariance (some nonzero values for realism)
    for (int i = 0; i < SIZE_STATE; ++i) {
        P_flat[i * SIZE_STATE + i] = 0.1;
    }

    // Input IMU measurements
    y_imu_t imu = {.array = {0.01, 0.02, -0.01, 0.3, -0.2, 0.5, 0.2, -0.1, 0.4, 1013.25}};

    // Bias vector for IMU (mapping to y_imu_t)
    y_imu_t bias = {
        .array = {
            0.001,
            -0.002,
            0.003, // Accelerometer bias (X, Y, Z)
            0.0001,
            -0.0002,
            0.0003, // Gyroscope bias (X, Y, Z)
            0.001,
            -0.002,
            0.003, // Magnetometer bias (X, Y, Z)
            0.005 // Barometer bias
        }
    };

    static double R_MTI_arr[SIZE_IMU_MEAS * SIZE_IMU_MEAS] = {};
    static arm_matrix_instance_f64 R_MTI = {
        .numRows = SIZE_IMU_MEAS, .numCols = SIZE_IMU_MEAS, .pData = R_MTI_arr
    };
    const double R_MTI_diag[SIZE_IMU_MEAS] = {1e-5, 1e-5, 1e-5, 5e-3, 5e-3, 5e-3, 2e1};
    math_init_matrix_diag(&R_MTI, (uint16_t)SIZE_IMU_MEAS, R_MTI_diag);

    // Expected corrected state (xhat) and covariance (Phat)
    x_state_t expected_state = {
        .array = {
            0.0003,
            0.0007,
            0.0001,
            0.0007,
            0.0003,
            -0.0002,
            0.0005,
            -0.0004,
            0.0084,
            -0.0063,
            8.2420,
            0.0015,
            0.0607
        }
    };

    double expected_P_flat[169] = {
        0.0999273827655033,
        2.81411893318334e-05,
        -1.62279719414715e-05,
        -4.23732815843498e-05,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        2.81411893318334e-05,
        0.0999082979348584,
        -1.16328046428393e-05,
        -3.0374720176036e-05,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        -1.62279719414715e-05,
        -1.16328046428393e-05,
        0.0998948335032559,
        1.7515965687675e-05,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        -4.23732815843498e-05,
        -3.0374720176036e-05,
        1.7515965687675e-05,
        0.0999338616953756,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        9.99900009999e-06,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        9.99900009999e-06,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        9.99900009999e-06,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0.1,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0.1,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0.1,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0.0581225548957822,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0.1,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0.1
    };

    // Act: Run the EKF correction step
    // y_meas is IMU_1(4:end)
    ekf_matrix_correct(&state, P_flat, &R_MTI, SIZE_IMU_MEAS, &imu.array[3], &bias.array[0]);

    // Assert
    double tolerance = 1e-5;

    // Check state (x)
    for (int i = 0; i < SIZE_STATE; i++) {
        EXPECT_NEAR(
            state.array[i], expected_state.array[i], abs(expected_state.array[i] * tolerance)
        );
    }

    // Check covariance (P)
    for (int i = 0; i < SIZE_STATE * SIZE_STATE; i++) {
        EXPECT_NEAR(P_flat[i], expected_P_flat[i], abs(expected_P_flat[i] * tolerance));
    }
}
