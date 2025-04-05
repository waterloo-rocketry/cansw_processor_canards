#include "fff.h"
#include <cmath>
#include <gtest/gtest.h>

extern "C" {
#include "application/estimator/model/quaternion.h"
}

using namespace std;

// helper functions to compare quaternions, 3x3 matrices, and 3d vectors
bool compare_quaternion(quaternion_t q1, quaternion_t q2) {
    for (int i = 0; i < 4; i++) {
        if (abs(q1.array[i] - q2.array[i]) >
            0.00001) { // tolerance to ensure that each entry matches
            return false;
        }
    }
    return true;
}

bool compare_matrix3d(matrix3d_t m1, matrix3d_t m2) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            if (abs(m1.array[i][j] - m2.array[i][j]) > 0.00001) {
                return false;
            }
        }
    }
    return true;
}

bool compare_vector3d(vector3d_t v1, vector3d_t v2) {
    for (int i = 0; i < 3; i++) {
        if (abs(v1.array[i] - v2.array[i]) > 0.00001) {
            return false;
        }
    }
    return true;
}

// unit tests for quaternion functions to compare C results to matlab model outputs
TEST(quaternion_norm_test, HandlesExpectedInput) {
    // ARRANGE: initialize quaternions with norms 0, 1, and 2
    quaternion_t q1 = {.array = {0, 0, 0, 0}};
    quaternion_t q2 = {.array = {1, 0, 0, 0}};
    quaternion_t q3 = {.array = {1, 1, 1, 1}};

    // ACT + ASSERT: compare values
    EXPECT_FLOAT_EQ(quaternion_norm(&q1), 0.0);
    EXPECT_FLOAT_EQ(quaternion_norm(&q2), 1.0);
    EXPECT_FLOAT_EQ(quaternion_norm(&q3), 2.0);
}

TEST(quaternion_normalize_test, HandlesExpectedInput) {
    // ARRANGE:
    quaternion_t q1 = {.array = {1, 1, 1, 1}};
    quaternion_t expectedResult = {.array = {0.5, 0.5, 0.5, 0.5}}; // expected normalized quaternion

    // ACT:
    quaternion_t actualResult = quaternion_normalize(&q1);

    // ASSERT:
    EXPECT_FLOAT_EQ(
        quaternion_norm(&actualResult), 1.0
    ); // norm of a normalized quaternion should be 1
    EXPECT_TRUE(compare_quaternion(actualResult, expectedResult));
}

TEST(quaternion_multiply_test, HandlesExpectedInput) {
    // ARRANGE:
    quaternion_t q1 = {.array = {1, 1, 1, 1}};
    quaternion_t q2 = {.array = {2, 2, 2, 2}};

    quaternion_t expectedResult = {.array = {-4, 4, 4, 4}}; // expected output

    // ACT: actual output
    quaternion_t actualResult = quaternion_multiply(&q1, &q2);

    // ASSERT:
    EXPECT_TRUE(compare_quaternion(actualResult, expectedResult));
}

// see if rotmatrix handles expected inputs
TEST(quaterion_rotmatrix_test, HandlesExpectedInput) {
    // ARRANGE:
    quaternion_t q = {.array = {1, 0, 0, 0}}; // test case
    matrix3d_t expectedResult = {.array = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

    // ACT:
    matrix3d_t actualResult = quaternion_rotmatrix(&q);

    // ASSERT:
    EXPECT_TRUE(compare_matrix3d(actualResult, expectedResult));
}

TEST(quaternion_derivative_test, HandlesExpectedInput) {
    //  ARRANGE:
    quaternion_t q1 = {.array = {1, 1, 1, 1}};
    vector3d_t v1 = {.array = {1, 1, 1}};

    quaternion_t expectedResult = {.array = {-0.7500, 0.25, 0.25, 0.25}};

    //  ACT:
    quaternion_t actualResult = quaternion_derivative(&q1, &v1);

    //  ASSERT:
    EXPECT_TRUE(compare_quaternion(actualResult, expectedResult));
}

TEST(quaternion_increment_test, HandlesExpectedInput) {
    //  ARRANGE:
    quaternion_t q1 = {.array = {1, 1, 1, 1}};
    vector3d_t v1 = {.array = {1, 1, 1}};
    float deltaT = 1.0;

    quaternion_t expectedResult = {.array = {-0.3358, 0.5438, 0.5438, 0.5438}};

    //  ACT:
    quaternion_t actualResult = quaternion_increment(&q1, &v1, deltaT);

    //  ASSERT:
    EXPECT_TRUE(compare_quaternion(actualResult, expectedResult));
}

TEST(quaternion_to_euler_test, HandlesExpectedInput) {
    //  ARRANGE:
    quaternion_t q1 = {.array = {1, 1, 1, 1}}; // sample data
    vector3d_t expectedResult = {.array = {1.5708, 0, 1.5708}};

    //    ACT:
    vector3d_t actualResult = quaternion_to_euler(&q1);
    //    ASSERT:
    EXPECT_TRUE(compare_vector3d(actualResult, expectedResult));
}

TEST(quaternion_to_roll_test, HandlesExpectedInput) {}

