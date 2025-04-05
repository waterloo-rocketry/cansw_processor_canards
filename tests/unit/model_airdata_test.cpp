#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
#include "application/estimator/model/model_airdata.h"
}

// Model airdata unit tests to compare C model results to matlab model results
TEST(ModelAirdataTest, model_altdata_test) {
    EXPECT_FLOAT_EQ(
        model_altdata(0), 4.4331e+04
    ); // expected return values for tests of 0, 1, and 100 cases taken from matlab model
    EXPECT_FLOAT_EQ(model_altdata(1), 3.9392e+04);
    EXPECT_FLOAT_EQ(model_altdata(100), 3.2461e+04);
}

TEST(ModelAirdataTest, model_airdata_test) {
    //  ARRANGE:
    float sample_altitude = 100;

    estimator_airdata_t expectedResult;
    expectedResult.pressure = 1.0013e+05;
    expectedResult.temperature = 287.5000;
    expectedResult.density = 1.2133;
    expectedResult.mach_local = 339.9129;

    //   ACT:
    estimator_airdata_t actualResult = model_airdata(sample_altitude);

    //   ASSERT:
    EXPECT_FLOAT_EQ(actualResult.pressure, expectedResult.pressure);
    EXPECT_FLOAT_EQ(actualResult.temperature, expectedResult.temperature);
    EXPECT_FLOAT_EQ(actualResult.density, expectedResult.density);
    EXPECT_FLOAT_EQ(actualResult.mach_local, expectedResult.mach_local);
}
