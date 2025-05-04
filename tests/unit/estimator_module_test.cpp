#include "fff.h"
#include <gtest/gtest.h>

#include "utils/mock_helpers.hpp"

extern "C" {
#include "application/estimator/estimator_module.h"
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/model_dynamics.h"
#include "application/estimator/model/quaternion.h"
#include "arm_math.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include "third_party/rocketlib/include/common.h"
#include <math.h>
#include <stdlib.h>

DEFINE_FFF_GLOBALS;

FAKE_VALUE_FUNC(w_status_t, log_text, uint32_t, const char *, const char *);
}

#define TOLERANCE 0.00001 // tolerance for float comparisons

class EstimatorModuleTest : public ::testing::Test {
protected:
    void SetUp() override {
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// clang-format off
/**
clear estimator_module;

% pad filter phase
flight_phase = 1;

% --- Global IMU selection flag (used inside estimator_module) ---
global IMU_select;
IMU_select = [1 1];

% --- Non-zero test inputs (example from your data) ---
timestamp = 0.005;
IMU_1 = [0.01; 0.02; -9.81; 0.001; -0.002; 0.0005; 0.3; 0.0; 0.4; 1013.25];
IMU_2 = [-0.02; 0.01; -9.78; 0.0005; 0.001; -0.001; 0.31; -0.01; 0.39; 1013.30];
cmd = [0; 0.005; 0; 0];   % Commanded angle and dummy timestamp
encoder = 0.03;             % radians
AHRS = [];                  % unused

% --- Call estimator_module ---
[xhat, Phat, controller_input, bias_1, bias_2] = estimator_module( ...
    timestamp, IMU_1, IMU_2, cmd, encoder, AHRS);

% --- Display the outputs in C-style formatting ---
fprintf('\n--- C-Style Outputs for Unit Tests ---\n');

% xhat
fprintf('\nxhat[] = {');
fprintf('%.6f, ', xhat(1:end-1));
fprintf('%.6f};\n', xhat(end));

% controller_input
fprintf('\ncontroller_input[] = {');
fprintf('%.6f, ', controller_input(1:end-1));
fprintf('%.6f};\n', controller_input(end));

% bias_1
fprintf('\nbias_1[] = {');
fprintf('%.6f, ', bias_1(1:end-1));
fprintf('%.6f};\n', bias_1(end));

% bias_2
fprintf('\nbias_2[] = {');
fprintf('%.6f, ', bias_2(1:end-1));
fprintf('%.6f};\n', bias_2(end));

% Phat (flattened row-major)
fprintf('\nPhat[] = {');
for i = 1:size(Phat, 1)
    for j = 1:size(Phat, 2)
        fprintf('%.6f', Phat(i, j));
        if ~(i == size(Phat, 1) && j == size(Phat, 2))
            fprintf(', ');
        end
    end
end
fprintf('};\n');
 */
// clang-format on
TEST(EstimatorModuleTest, BothImusAlivePadFilterPhaseOnce) {
    // Arrange
    estimator_module_input_t input = {0};
    input.timestamp = 0.005;
    input.movella = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25};
    input.pololu = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30};
    input.movella_is_dead = false;
    input.pololu_is_dead = false;

    // Commanded angle and timestamp for the controller
    controller_output_t cmd = {0, 5}; // commanded_angle in radians, timestamp in ms
    input.cmd = cmd;
    input.encoder = 0.03;

    flight_phase_state_t flight_phase = STATE_SE_INIT;

    // Initializing the context
    estimator_module_ctx_t ctx = {0};
    controller_input_t controller_input = {0}; // Initialize controller_input_t to zero

    // Act
    w_status_t status = estimator_module(&input, flight_phase, &ctx, &controller_input);

    // Expected output values for x_state_t
    x_state_t expect_x_init = {
        .array = {
            0.573781,
            -0.413347,
            0.573488,
            0.413558,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            25762.833869,
            3.627599,
            0.000000
        }
    };

    // Expected biases for Movella and Pololu IMUs
    y_imu_t expect_bias_1 = {
        .array = {0.0, 0.0, 0.0, 0.001000, -0.002000, 0.000500, 0.126540, 0.379619, -0.299796, 0.0}
    };

    y_imu_t expect_bias_2 = {
        .array = {0.0, 0.0, 0.0, 0.000500, 0.001000, -0.001000, 0.132866, 0.366974, -0.309801, 0.0}
    };

    // ignore controller output for pad filter phase

    // Assert: Verify x_state_t (for state estimation module)
    for (int i = 0; i < 13; ++i) {
        EXPECT_NEAR(
            ctx.x.array[i], expect_x_init.array[i], fabs(expect_x_init.array[i] * TOLERANCE)
        );
    }

    // Assert: Verify bias_1 and bias_2
    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(
            ctx.bias_movella.array[i],
            expect_bias_1.array[i],
            std::fabs(expect_bias_1.array[i] * TOLERANCE)
        );
        EXPECT_NEAR(
            ctx.bias_pololu.array[i],
            expect_bias_2.array[i],
            std::fabs(expect_bias_2.array[i] * TOLERANCE)
        );
    }

    // Optionally, check Phat
    double expected_Phat[169] = {0}; // Set the expected values for P matrix if necessary
    for (int i = 0; i < 169; ++i) {
        EXPECT_NEAR(ctx.P_flat[i], expected_Phat[i], 1e-5); // Adjust tolerance if needed
    }
}

TEST(EstimatorModuleTest, BothImusAlivePadFilterPhaseTwice) {
    // Arrange
    estimator_module_input_t input = {0};
    input.timestamp = 0.005;
    input.movella = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25};
    input.pololu = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30};
    input.movella_is_dead = false;
    input.pololu_is_dead = false;

    // Commanded angle and timestamp for the controller
    controller_output_t cmd = {0.1, 5}; // commanded_angle in radians, timestamp in ms
    input.cmd = cmd;
    input.encoder = 0.03;

    flight_phase_state_t flight_phase = STATE_SE_INIT;

    // Initializing the context
    estimator_module_ctx_t ctx = {0};
    controller_input_t controller_input = {0}; // Initialize controller_input_t to zero

    // Act
    w_status_t status = estimator_module(&input, flight_phase, &ctx, &controller_input);

    // Expected output values for x_state_t
    x_state_t expect_x_init = {
        .array = {
            0.573781,
            -0.413347,
            0.573488,
            0.413558,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            25762.842421,
            3.627599,
            0.0
        }
    };

    // Expected biases for Movella and Pololu IMUs
    y_imu_t expect_bias_1 = {
        .array = {0.0, 0.0, 0.0, 0.001000, -0.002000, 0.000500, 0.126540, 0.379619, -0.299796, 0.0}
    };

    y_imu_t expect_bias_2 = {
        .array = {0.0, 0.0, 0.0, 0.000500, 0.001000, -0.001000, 0.132866, 0.366974, -0.309801, 0.0}
    };

    // ignore controller output for pad filter phase

    // Assert: Verify x_state_t (for state estimation module)
    for (int i = 0; i < 13; ++i) {
        EXPECT_NEAR(
            ctx.x.array[i], expect_x_init.array[i], fabs(expect_x_init.array[i] * TOLERANCE)
        );
    }

    // Assert: Verify bias_1 and bias_2
    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(
            ctx.bias_movella.array[i],
            expect_bias_1.array[i],
            std::fabs(expect_bias_1.array[i] * TOLERANCE)
        );
        EXPECT_NEAR(
            ctx.bias_pololu.array[i],
            expect_bias_2.array[i],
            std::fabs(expect_bias_2.array[i] * TOLERANCE)
        );
    }

    // Optionally, check Phat
    double expected_Phat[169] = {0}; // Set the expected values for P matrix if necessary
    for (int i = 0; i < 169; ++i) {
        EXPECT_NEAR(ctx.P_flat[i], expected_Phat[i], 1e-5); // Adjust tolerance if needed
    }
}
