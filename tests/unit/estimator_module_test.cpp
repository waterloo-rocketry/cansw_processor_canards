/**
 * test outputs from simulink-canards commit 2c8c534
 */

#include "fff.h"
#include <gtest/gtest.h>

#include "utils/mock_helpers.hpp"

extern "C" {
#include "application/estimator/ekf.h"
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

#define TOLERANCE 0.001 // tolerance for float comparisons

class EstimatorModuleTest : public ::testing::Test {
protected:
    void SetUp() override {
        ekf_init();
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

// clang-format off
/**
clear estimator_module;
clear pad_filter;
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
TEST_F(EstimatorModuleTest, BothImusAlivePadFilterPhaseOnce) {
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
            0.413347,
            -0.573781,
            -0.413558,
            -0.573488,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            250.000000,
            3.482826,
            0.000000
        }
    };

    // Expected biases for Movella and Pololu IMUs
    y_imu_t expect_bias_1 = {
        .array = {
            0.010000,
            0.020000,
            -9.810000,
            0.001000,
            -0.002000,
            0.000500,
            0.126540,
            0.379619,
            0.299796,
            -97343.221211
        }
    };

    y_imu_t expect_bias_2 = {
        .array = {
            -0.020000,
            0.010000,
            -9.780000,
            0.000500,
            0.001000,
            -0.001000,
            0.113892,
            0.373299,
            0.309801,
            -97343.171211
        }
    };

    // ignore controller output for pad filter phase

    // Assert: Verify x_state_t (for state estimation module)
    for (int i = 0; i < 13; ++i) {
        EXPECT_NEAR(
            ctx.x.array[i], expect_x_init.array[i], abs(expect_x_init.array[i] * TOLERANCE)
        );
    }

    // Assert: Verify bias_1 and bias_2
    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(
            ctx.bias_movella.array[i],
            expect_bias_1.array[i],
            std::abs(expect_bias_1.array[i] * TOLERANCE)
        );
        EXPECT_NEAR(
            ctx.bias_pololu.array[i],
            expect_bias_2.array[i],
            std::abs(expect_bias_2.array[i] * TOLERANCE)
        );
    }

    // Optionally, check Phat
    double expected_Phat[169] = {0}; // Set the expected values for P matrix if necessary
    for (int i = 0; i < 169; ++i) {
        EXPECT_NEAR(ctx.P_flat[i], expected_Phat[i], 1e-5); // Adjust tolerance if needed
    }
}

// clang-format off
/**
clear all;
clear pad_filter; clear estimator_module;
format long g

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
estimator_module( ...
    timestamp, IMU_1, IMU_2, cmd, encoder, AHRS);

% call again
timestamp = 0.01
cmd = [0; 0.1; 0; 0];   % Commanded angle and dummy timestamp
IMU_1 = [0.06; 0.1; -9.99; 0.001; -0.002; 0.0005; 0.5; 0.4; 0.4; 1010.25];
IMU_2 = [-0.03; 0.08; -9.1; 0.005; 0.01; -0.001; 0.31; -0.01; 0.99; 1014.30];
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
TEST_F(EstimatorModuleTest, BothImusAlivePadFilterPhaseTwice) {
    // Arrange - First iteration
    estimator_module_input_t input = {0};
    input.timestamp = 0.005;
    input.movella = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25};
    input.pololu = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30};
    input.movella_is_dead = false;
    input.pololu_is_dead = false;

    controller_output_t cmd = {0.1, 5}; // Commanded angle in radians, timestamp in ms
    input.cmd = cmd;
    input.encoder = 0.03;

    flight_phase_state_t flight_phase = STATE_SE_INIT;

    estimator_module_ctx_t ctx = {0};
    controller_input_t controller_input = {0};

    // Act - First call
    w_status_t status = estimator_module(&input, flight_phase, &ctx, &controller_input);

    // --- Expected outputs from first iteration ---
    x_state_t expect_x_init = {
        .array = {
            0.571031,
            -0.417133,
            0.570745,
            0.417342,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            250.000000,
            4.481753,
            0.000000
        }
    };

    y_imu_t expect_bias_1 = {
        .array = {
            0.000000,
            0.000000,
            0.000000,
            0.001000,
            -0.002000,
            0.000500,
            0.119601,
            0.381864,
            -0.300800,
            -97343.236211
        }
    };

    y_imu_t expect_bias_2 = {
        .array = {
            0.000000,
            0.000000,
            0.000000,
            0.000522,
            0.001045,
            -0.001000,
            0.128910,
            0.371555,
            -0.309803,
            -97343.166211
        }
    };

    // Assert - First iteration
    // for (int i = 0; i < 13; ++i) {
    //     EXPECT_NEAR(
    //         ctx.x.array[i], expect_x_init.array[i], abs(expect_x_init.array[i] * TOLERANCE)
    //     );
    // }

    // for (int i = 0; i < 10; ++i) {
    //     EXPECT_NEAR(
    //         ctx.bias_movella.array[i],
    //         expect_bias_1.array[i],
    //         abs(expect_bias_1.array[i] * TOLERANCE)
    //     );
    //     EXPECT_NEAR(
    //         ctx.bias_pololu.array[i],
    //         expect_bias_2.array[i],
    //         abs(expect_bias_2.array[i] * TOLERANCE)
    //     );
    // }

    // --- Second iteration setup ---
    input.timestamp = 0.01;
    input.movella = {0.06, 0.1, -9.99, 0.001, -0.002, 0.0005, 0.5, 0.4, 0.4, 1010.25};
    input.pololu = {-0.03, 0.08, -9.1, 0.005, 0.01, -0.001, 0.31, -0.01, 0.99, 1014.30};
    input.cmd = {0.0, 10}; // Dummy cmd.angle = 0, timestamp = 100ms

    // Act - Second call
    status = estimator_module(&input, flight_phase, &ctx, &controller_input);

    // --- Expected outputs from second iteration ---
    x_state_t expect_x_2 = {
        .array = {
            0.417133,
            -0.571031,
            -0.417342,
            -0.570745,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            0.000000,
            250.000000,
            3.482826,
            0.000000
        }
    };

    y_imu_t expect_bias_1_2 = {
        .array = {
            0.010250,
            0.020400,
            -9.810900,
            0.001000,
            -0.002000,
            0.000500,
            0.123412,
            0.380649,
            0.300800,
            -97343.236211
        }
    };

    y_imu_t expect_bias_2_2 = {
        .array = {
            -0.020050,
            0.010350,
            -9.776600,
            0.000522,
            0.001045,
            -0.001000,
            0.109854,
            0.377628,
            0.309803,
            -97343.166211
        }
    };

    double expected_Phat[169] = {0}; // All zeros from MATLAB output

    // Assert - Second iteration
    for (int i = 0; i < 13; ++i) {
        EXPECT_NEAR(ctx.x.array[i], expect_x_2.array[i], abs(expect_x_2.array[i] * TOLERANCE));
    }

    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(
            ctx.bias_movella.array[i],
            expect_bias_1_2.array[i],
            abs(expect_bias_1_2.array[i] * TOLERANCE)
        );
        EXPECT_NEAR(
            ctx.bias_pololu.array[i],
            expect_bias_2_2.array[i],
            abs(expect_bias_2_2.array[i] * TOLERANCE)
        );
    }

    for (int i = 0; i < 169; ++i) {
        EXPECT_NEAR(ctx.P_flat[i], expected_Phat[i], 1e-5); // Adjust tolerance if needed
    }
}

// clang-format off
/** NOTE: to run this script, edit matlab estimator_module to replace the `persistent` variables
 * with `global` variables. Ie, delete 'persistent' and write 'global' in its place.
clear all;
clear pad_filter; clear estimator_module;
format long g
    global P
    global b; % remembers g_x, g_P, g_t from last iteration
    global IMU_select 
    global flight_phase
    global x
    global t

P = zeros(13);
b.bias_1 = zeros(10, 1);
b.bias_2 = zeros(10, 1);
t = 0.005;

global IMU_select;
IMU_select = [1 1];

% --- Non-zero test inputs (example from your data) ---
IMU_1 = [0.01; 0.02; -9.81; 0.001; -0.002; 0.0005; 0.3; 0.0; 0.4; 1013.25];
IMU_2 = [-0.02; 0.01; -9.78; 0.0005; 0.001; -0.001; 0.31; -0.01; 0.39; 1013.30];
cmd = 0.1;   % Commanded angle and dummy timestamp
encoder = 0.03;             % radians
AHRS = [];                  % unused

% in flight now
flight_phase = 0;

timestamp = 0.1;
cmd = 0.1;
x = [...
	0.573781;
         -0.413347;
         0.573488;
         0.413558;
         0.000000;
         0.000000;
         0.000000;
         0.000000;
         0.000000;
         0.000000;
         25762.842421;
         3.627599;
         0.000000]

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
TEST_F(EstimatorModuleTest, BothImusAliveActAllowedPhaseOnce) {
    // Arrange
    estimator_module_input_t input = {0};
    input.timestamp = 0.1;
    input.movella = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25};
    input.pololu = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30};
    input.movella_is_dead = false;
    input.pololu_is_dead = false;

    controller_output_t cmd = {0.1, 5}; // commanded_angle, timestamp (ms)
    input.cmd = cmd;
    input.encoder = 0.03;

    flight_phase_state_t flight_phase = STATE_ACT_ALLOWED;

    estimator_module_ctx_t ctx = {
        .x =
            {.array =
                 {0.573781,
                  -0.413347,
                  0.573488,
                  0.413558,
                  0.000000,
                  0.000000,
                  0.000000,
                  0.000000,
                  0.000000,
                  0.000000,
                  25762.842421,
                  3.627599,
                  0.000000}},
        .t = 0.005
    };
    controller_input_t controller_input = {0};

    // Act
    w_status_t status = estimator_module(&input, flight_phase, &ctx, &controller_input);

    // Expected x_state

    x_state_t expect_x_init = {
        {0.573781,
         -0.413347,
         0.573488,
         0.413558,
         0.000857013982,
         -0.001143,
         0.00007141783,
         -0.00062567116,
         0.885550,
         -1.225233,
         25762.845607,
         3.626911,
         0.033586}
    };

    y_imu_t expect_bias_1 = {
        {0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000}
    };

    y_imu_t expect_bias_2 = {
        {0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000,
         0.000000}
    };

    double expected_Phat[169] = {
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000095,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000095, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000095, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000095, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 2.850000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
        0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.001919
    };

    // Assert: xhat
    for (int i = 0; i < 13; ++i) {
        if (expect_x_init.array[i] == 0) {
            EXPECT_NEAR(ctx.x.array[i], expect_x_init.array[i], TOLERANCE);
        } else {
            EXPECT_NEAR(
                ctx.x.array[i], expect_x_init.array[i], abs(expect_x_init.array[i] * TOLERANCE)
            );
        }
    }

    // Assert: bias_1 and bias_2
    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(
            ctx.bias_movella.array[i],
            expect_bias_1.array[i],
            abs(expect_bias_1.array[i] * TOLERANCE)
        );
        EXPECT_NEAR(
            ctx.bias_pololu.array[i],
            expect_bias_2.array[i],
            abs(expect_bias_2.array[i] * TOLERANCE)
        );
    }

    // Expected controller input
    // TODO: use array. this is just conveniently copied as an array from matlab output vector
    double expected_controller_input[5] = {-0.000410, 0.000857, 0.033586, 0.039296, 3.626911};

    // Assert: controller_input values
    EXPECT_NEAR(
        controller_input.roll_state.roll_angle,
        expected_controller_input[0],
        abs(expected_controller_input[0] * TOLERANCE)
    );
    EXPECT_NEAR(
        controller_input.roll_state.roll_rate,
        expected_controller_input[1],
        abs(expected_controller_input[1] * TOLERANCE)
    );
    EXPECT_NEAR(
        controller_input.roll_state.canard_angle,
        expected_controller_input[2],
        abs(expected_controller_input[2] * TOLERANCE)
    );
    EXPECT_NEAR(
        controller_input.pressure_dynamic,
        expected_controller_input[3],
        abs(expected_controller_input[3] * TOLERANCE)
    );
    EXPECT_NEAR(
        controller_input.canard_coeff,
        expected_controller_input[4],
        abs(expected_controller_input[4] * TOLERANCE)
    );

    // Assert: Phat matrix
    for (int i = 0; i < 169; ++i) {
        EXPECT_NEAR(ctx.P_flat[i], expected_Phat[i], 1e-6);
    }
}
