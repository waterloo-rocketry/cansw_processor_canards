#include <gtest/gtest.h>
#include "application/estimator/model/model_airdata.h"

//Model airdata unit tests to compare C model results to matlab model results
TEST(ModelAirdataTest, model_altdata_test) {
    EXPECT_FLOAT_EQ(model_altdata(0), 4.4331e+04); //expected return values for tests of 0, 1, and 100 cases taken from matlab model
    EXPECT_FLOAT_EQ(model_altdata(1), 3.9392e+04);
    EXPECT_FLOAT_EQ(model_altdata(100), 3.2461e+04);
}

TEST(ModelAirdataTest, model_airdata_test) {
    EXPECT_FLOAT_EQ(model_airdata(0), 101325);
    EXPECT_FLOAT_EQ(model_airdata(1), 1.0131e+05);
    EXPECT_FLOAT_EQ(model_airdata(100), 1.0013e+05);
}
