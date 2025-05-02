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

// Test case for estimator_module
TEST(EstimatorModuleTest, TestNominalCase) {
    // Arrange
    estimator_module_input_t input;
    input.timestamp = 0.005;
    input.movella = {0.01, 0.02, -9.81, 0.001, -0.002, 0.0005, 0.3, 0.0, 0.4, 1013.25};
    input.pololu = {-0.02, 0.01, -9.78, 0.0005, 0.001, -0.001, 0.31, -0.01, 0.39, 1013.30};
    input.movella_is_dead = false;
    input.pololu_is_dead = false;

    // Commanded angle and timestamp for the controller
    controller_output_t cmd = {0.1, 1000}; // commanded_angle in radians, timestamp in ms
    input.cmd = cmd;
    input.encoder = 0.03;

    flight_phase_state_t flight_phase = STATE_SE_INIT; // or STATE_ACT_ALLOWED for in-flight tests

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

    // Expected controller_input values from state estimation module (based on the array)
    controller_input_t expected_controller_input = {
        .roll_state = {0.0, 0.0, 0.0}, // roll_angle = 0.0, roll_rate = 0.0, canard_angle = 0.0
                                       // (from MATLAB output)
        .canard_coeff = 0.0, // Corresponding to the MATLAB output
        .pressure_dynamic = 3.627599 // Matching the last value from the MATLAB output
    };

    // tolerance = 0.00001 (for floating point comparison)

    // Assert: Verify x_state_t (for state estimation module)
    for (int i = 0; i < 13; ++i) {
        EXPECT_NEAR(
            ctx.x.array[i], expect_x_init.array[i], std::fabs(expect_x_init.array[i] * TOLERANCE)
        );
    }

    // Assert: Verify controller_input_t (as derived from state estimation module)
    // Assert: Verify roll_state (using union struct access)
    EXPECT_NEAR(
        controller_input.roll_state.roll_angle,
        expected_controller_input.roll_state.roll_angle,
        TOLERANCE
    );
    EXPECT_NEAR(
        controller_input.roll_state.roll_rate,
        expected_controller_input.roll_state.roll_rate,
        TOLERANCE
    );
    EXPECT_NEAR(
        controller_input.roll_state.canard_angle,
        expected_controller_input.roll_state.canard_angle,
        TOLERANCE
    );

    // Assert: Verify canard_coeff and pressure_dynamic
    EXPECT_NEAR(
        controller_input.canard_coeff, expected_controller_input.canard_coeff, TOLERANCE
    ); // Verify canard_coeff
    EXPECT_NEAR(
        controller_input.pressure_dynamic, expected_controller_input.pressure_dynamic, TOLERANCE
    ); // Verify pressure_dynamic

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

    // Optionally, check Phat (if it is relevant and available)
    double expected_Phat[169] = {0}; // Set the expected values for P matrix if necessary
    for (int i = 0; i < 169; ++i) {
        EXPECT_NEAR(ctx.P[i], expected_Phat[i], 1e-5); // Adjust tolerance if needed
    }
}

