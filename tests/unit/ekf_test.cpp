#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "application/estimator/ekf.h"
#include "application/estimator/estimator_types.h"
#include "arm_math.h"
DEFINE_FFF_GLOBALS;
}

class EstimatorEKFTest : public ::testing::Test {
protected:
    void SetUp() override {
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

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

    x_state_t expected_state = {
        .array = {
            0.268274165208879,
            0.696419511023482,
            0.194712315844408,
            0.636487196262266,
            0.643134076016596,
            0.006279518794539,
            0.471014707988784,
            -0.400684442488097,
            8.416935880119656,
            -6.314095921040009,
            0.525564693616073,
            1.457215728968408,
            75.599819909717795
        }
    };
    double expected_P_flat[SIZE_STATE * SIZE_STATE] = {
        9.15736E-09, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 9.15736E-09, 0, 0, 0, 0, 0, 0,
        0,           0, 0, 0, 0, 0, 0, 9.15736E-09, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0,
        9.15736E-09, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 0.915735525, 0, 0, 0, 0, 0, 0,
        0,           0, 0, 0, 0, 0, 0, 0.915735525, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0,
        0.915735525, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 0.018314711, 0, 0, 0, 0, 0, 0,
        0,           0, 0, 0, 0, 0, 0, 0.018314711, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0,
        0.018314711, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0, 0.009157355, 0, 0, 0, 0, 0, 0,
        0,           0, 0, 0, 0, 0, 0, 91.57355252, 0, 0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0,
        9.157355252
    };
    ;

    // Act
    ekf_matrix_predict(&state, P_flat, &input, dt);

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