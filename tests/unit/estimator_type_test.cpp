
#include <gtest/gtest.h>
#include <iostream>
#include <type_traits>
#include <cstddef>

extern "C" {
#include "application/estimator/estimator_types.h"
}

x_state_t state;

// Test fixture for estimator lib test
class EstimatorTypeTest : public ::testing::Test {
protected:
    void SetUp() override {}

    void TearDown() override {}
};


TEST_F(EstimatorTypeTest, SizeOfType) {
    // Arrange: set up the inputs and expected result
    std::cout << "sizeof(x_state_t): " << sizeof(x_state_t) << " bytes\n";

    // float is 4 byte, 13 states, 4*13 = 52
    EXPECT_EQ(sizeof(x_state_t), 52);
}

TEST_F(EstimatorTypeTest, OffsetsOfType) {
    // Arrange: set up the inputs and expected result
    std::cout << "Offset of attitude: " << offsetof(x_state_t, attitude) << "\n";
    std::cout << "Offset of rates: "    << offsetof(x_state_t, rates) << "\n";
    std::cout << "Offset of velocity: " << offsetof(x_state_t, velocity) << "\n";
    std::cout << "Offset of altitude: " << offsetof(x_state_t, altitude) << "\n";
    std::cout << "Offset of CL: "       << offsetof(x_state_t, CL) << "\n";
    std::cout << "Offset of delta: "    << offsetof(x_state_t, delta) << "\n";

    // if no padding, delta should begin at 12*4 = 48 bytes
    EXPECT_EQ(offsetof(x_state_t, delta), 48);
} 
