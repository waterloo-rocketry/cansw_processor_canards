#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "application/estimator/model/model_airdata.h"
#include <math.h>

#define TOLERANCE 0.000001 // non-ratio, checks 6 decimals points

DEFINE_FFF_GLOBALS;
}

class AirdataJacobianTest : public ::testing::Test {
protected:
    void SetUp() override {
        FFF_RESET_HISTORY();
    }

    void TearDown() override {}
};

TEST_F(AirdataJacobianTest, NominalCheck) {
    // Arrange
    double expected_output[10] = {
        -8.21305872243281,
        -4.94526529701412,
        -4.62642002367724,
        -5.06763960026881,
        -5.80066755868718,
        -4.81194010761366,
        -6.90097109449443,
        -5.33259093653041,
        -6.60104077020978,
        -4.30528973062648
    };

    double altitude[10] = {
        3800.02915415076,
        8355.83204114805,
        8914.36287142533,
        8148.73886737038,
        6982.12949533516,
        8585.94617828255,
        5425.86103721258,
        7713.08279333824,
        5830.17180344317,
        9507.35009011244
    }; // rand generated nums

    // Act and Assert
    for (int i = 0; i < 10; i++) {
        double actual_output = model_airdata_jacobian(altitude[i]);
        EXPECT_NEAR(expected_output[i], actual_output, TOLERANCE); // check 6 decimals for overlap
    }
}