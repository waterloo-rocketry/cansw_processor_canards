#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "application/estimator/model/model_airdata.h"
}

// 0.005% tolerance, proportional to the actual value
#define tolerance 0.00005

// Model airdata unit tests to compare C model results to matlab model results
TEST(ModelAirdataTest, model_altdata_test) {
    EXPECT_NEAR(
        model_altdata(100001), 110.762, abs(110.762 * tolerance)
    ); // expected return values for tests of 100001, 1, and 100 cases taken from matlab model
    EXPECT_NEAR(model_altdata(21345), 1.1346e+04, abs(1.1346e+04 * tolerance));
    EXPECT_NEAR(model_altdata(5474), 1.8828e+04, abs(1.8828e+04 * tolerance));
}

TEST(ModelAirdataTest, model_airdata_test) {
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
