#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "application/estimator/model/model_airdata.h"
// 0.005% tolerance, proportional to the actual value
#define tolerance 0.00005
#define TOLERANCE 0.000001 // non-ratio, checks 6 decimals points 

}

class ModelAirdataTest : public ::testing::Test {
    protected:
        void SetUp() override {
            FFF_RESET_HISTORY();
        }
    
        void TearDown() override {}
    };


// Model airdata unit tests to compare C model results to matlab model results
TEST_F(ModelAirdataTest, ModelAltdataCheck) {
    EXPECT_NEAR(
        model_altdata(100001), 110.762, abs(110.762 * tolerance)
    ); // expected return values for tests of 100001, 1, and 100 cases taken from matlab model
    EXPECT_NEAR(model_altdata(21345), 1.1346e+04, abs(1.1346e+04 * tolerance));
    EXPECT_NEAR(model_altdata(5474), 1.8828e+04, abs(1.8828e+04 * tolerance));
}

TEST_F(ModelAirdataTest, ModelAirdataNonimalCheck) {
    //  ARRANGE:
    double sample_altitude = 100;

    estimator_airdata_t expectedResult;
    expectedResult.pressure = 1.0013e+05;
    expectedResult.temperature = 287.5000;
    expectedResult.density = 1.2133;
    expectedResult.mach_local = 339.9129;

    //   ACT:
    estimator_airdata_t actualResult = model_airdata(sample_altitude);

    //   ASSERT:
    EXPECT_NEAR(
        actualResult.pressure, expectedResult.pressure, abs(expectedResult.pressure * tolerance)
    );
    EXPECT_NEAR(
        actualResult.temperature,
        expectedResult.temperature,
        abs(expectedResult.temperature * tolerance)
    );
    EXPECT_NEAR(
        actualResult.density, expectedResult.density, abs(expectedResult.density * tolerance)
    );
    EXPECT_NEAR(
        actualResult.mach_local,
        expectedResult.mach_local,
        abs(expectedResult.mach_local * tolerance)
    );
}

TEST_F(ModelAirdataTest, JacobianCheck) {
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
