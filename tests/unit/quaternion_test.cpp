#include <gtest/gtest.h>
#include "quaternion.h"
#include "common/math/math.h"

bool compare_quaternion(quaternion_t q1, quaternion_t q2) {
    for (int i = 0; i < 4; i++) {
        if (q1.array[i] - q2.array[i] > 0.00001) { //tolerance to ensure that each entry matches
            return false;
        }
    }
    return true;
}

bool compare_matrix3d(matrix3d_t m1, matrix3d_t m2) {
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      if (m1.array[i][j] - m2.array[i][j] > 0.00001) {
        return false;
      }
    }
  }
  return true;
}

//unit tests for quaternion functions to compare C results to matlab model outputs
TEST(quaternion_norm_test, HandlesExpectedInput) {
    //initialize quaternions with norms 0, 1, and 2
    quaternion_t q1 = {.array = {0,0,0,0}};
    quaternion_t q2 = {.array = {1,0,0,0}};
    quaternion_t q3 = {.array = {1,1,1,1}};

    //compare values
    EXPECT_FLOAT_EQ(quaternion_norm(&q1), 0.0);
    EXPECT_FLOAT_EQ(quaternion_norm(&q2),1.0);
    EXPECT_FLOAT_EQ(quaternion_norm(&q3), 2.0);
}

TEST(quaternion_normalize_test, HandlesExpectedInput){
    quaternion_t q1 = {.array = {1,1,1,1}};

    //expected normalized quaternion
    quaternion_t q1Normalized = {.array = {0.5,0.5,0.5,0.5}};

    EXPECT_FLOAT_EQ(quaternion_norm(&q1), 1.0); //norm of a normalized quaternion should be 1
    EXPECT_TRUE(compare_quaternion(quaternion_normalize(&q1), q1Normalized));
}

TEST(quaternion_multiply_test, HandlesExpectedInput) {
    quaternion_t q1 = {.array = {1,1,1,1}};
    quaternion_t q2 = {.array = {2,2,2,2}};

    //expected output
    quaternion_t expectedResult = {.array = {-4,4,4,4}};

    //actual output
    quaternion_t actualResult = quaternion_multiply(&q1, &q2);

    EXPECT_TRUE(compare_quaternion(actualResult, expectedResult));
}

//see if rotmatrix handles expected inputs
TEST(quaterion_rotmatrix_test, HandlesExpectedInput) {
    quaternion_t q = {.array = {1,0,0,0}};//test case
    matrix3d_t actualResult = quaternion_rotmatrix(&q);
    matrix3d_t expectedResult = {.array = {{1,0,0},{0,1,0},{0,0,1}}};

    EXPECT_TRUE(compare_matrix3d(actualResult, expectedResult));
}

TEST(quaternion_derivative_test, HandlesExpectedInput) {
  quaternion_t q1 = {.array = {1,1,1,1}};
  vector3d_t v1 = {.array = {1,1,1}};

  quaternion_t expectedResult = {.array = {-0.7500,0.25,0.25,0.25}};
  quaternion_t actualResult = quaternion_derivative(&q1, &v1);

  EXPECT_TRUE(compare_quaternion(actualResult, expectedResult));
}

TEST(quaternion_increment_test, HandlesExpectedInput) {
  quaternion_t q1 = {.array = {1,1,1,1}};
  vector3d_t v1 = {.array = {1,1,1}};
  float deltaT = 1.0;

  quaternion_t expectedResult = {.array = {-0.3358,0.5438,0.5438,0.5438}};
  quaternion_t actualResult = quaternion_increment(&q1, &v1, deltaT);
  EXPECT_TRUE(compare_quaternion(actualResult, expectedResult));
}

TEST(quaternion_to_euler_test, HandlesExpectedInput) {
  
}


