#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
}

class CommonMathTest : public ::testing::Test {
protected:
    void SetUp() override {}

    void TearDown() override {}
};

// Helper function to compare vectors
bool vectors_are_equal(const vector3d_t &a, const vector3d_t &b, float tolerance = 1e-5f) {
    return (fabs(a.x - b.x) < tolerance) && (fabs(a.y - b.y) < tolerance) &&
           (fabs(a.z - b.z) < tolerance);
}

// Helper function to compare matrices
bool matrices_are_equal(const matrix3d_t &a, const matrix3d_t &b, float tolerance = 1e-5f) {
    return (fabs(a.s11 - b.s11) < tolerance) && (fabs(a.s12 - b.s12) < tolerance) &&
           (fabs(a.s13 - b.s13) < tolerance) && (fabs(a.s21 - b.s21) < tolerance) &&
           (fabs(a.s22 - b.s22) < tolerance) && (fabs(a.s23 - b.s23) < tolerance) &&
           (fabs(a.s31 - b.s31) < tolerance) && (fabs(a.s32 - b.s32) < tolerance) &&
           (fabs(a.s33 - b.s33) < tolerance);
}

// Test case: Basic test with a known matrix
TEST(CommonMathTest, BasicTest) {
    matrix3d_t input = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f};

    matrix3d_t expected = {1.0f, 4.0f, 7.0f, 2.0f, 5.0f, 8.0f, 3.0f, 6.0f, 9.0f};

    matrix3d_t result = math_matrix3d_transp(&input);

    EXPECT_TRUE(matrices_are_equal(result, expected));
}

// Test case: Identity matrix should remain the same after transpose
TEST(CommonMathTest, IdentityMatrix) {
    matrix3d_t input = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

    matrix3d_t expected = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

    matrix3d_t result = math_matrix3d_transp(&input);

    EXPECT_TRUE(matrices_are_equal(result, expected));
}

// Test case: Zero matrix should remain zero after transpose
TEST(CommonMathTest, ZeroMatrix) {
    matrix3d_t input = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    matrix3d_t expected = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    matrix3d_t result = math_matrix3d_transp(&input);

    EXPECT_TRUE(matrices_are_equal(result, expected));
}

// Test case: Symmetric matrix should remain the same after transpose
TEST(CommonMathTest, SymmetricMatrix) {
    matrix3d_t input = {1.0f, 2.0f, 3.0f, 2.0f, 4.0f, 5.0f, 3.0f, 5.0f, 6.0f};

    matrix3d_t expected = {1.0f, 2.0f, 3.0f, 2.0f, 4.0f, 5.0f, 3.0f, 5.0f, 6.0f};

    matrix3d_t result = math_matrix3d_transp(&input);

    EXPECT_TRUE(matrices_are_equal(result, expected));
}

// Test case: Non-symmetric matrix should transpose into a different matrix
TEST(CommonMathTest, NonSymmetricMatrix) {
    matrix3d_t input = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f};

    matrix3d_t expected = {1.0f, 4.0f, 7.0f, 2.0f, 5.0f, 8.0f, 3.0f, 6.0f, 9.0f};

    matrix3d_t result = math_matrix3d_transp(&input);

    EXPECT_TRUE(matrices_are_equal(result, expected));
}

// Test case: Basic rotation test with a known matrix and vector
TEST(CommonMathTest, BasicRotationTest) {
    matrix3d_t rotation_matrix = {
        0.0f,
        -1.0f,
        0.0f, // 90-degree rotation around the z-axis
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        1.0f
    };

    vector3d_t input_vector = {1.0f, 0.0f, 0.0f};
    vector3d_t expected_result = {0.0f, 1.0f, 0.0f}; // Expected result after 90-degree rotation

    vector3d_t result = math_vector3d_rotate(&rotation_matrix, &input_vector);

    EXPECT_TRUE(vectors_are_equal(result, expected_result));
}

// Test case: Identity matrix should return the same vector
TEST(CommonMathTest, IdentityMatrixTest) {
    matrix3d_t identity_matrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

    vector3d_t input_vector = {3.0f, -4.0f, 5.0f};

    // After rotating with the identity matrix, the result should be the same vector
    vector3d_t expected_result = {3.0f, -4.0f, 5.0f};

    vector3d_t result = math_vector3d_rotate(&identity_matrix, &input_vector);

    EXPECT_TRUE(vectors_are_equal(result, expected_result));
}

// Test case: Zero vector should remain zero after rotation
TEST(CommonMathTest, ZeroVectorTest) {
    matrix3d_t rotation_matrix = {
        0.0f,
        -1.0f,
        0.0f, // Example rotation matrix
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        1.0f
    };

    vector3d_t zero_vector = {0.0f, 0.0f, 0.0f};

    // Rotating the zero vector should result in the zero vector
    vector3d_t expected_result = {0.0f, 0.0f, 0.0f};

    vector3d_t result = math_vector3d_rotate(&rotation_matrix, &zero_vector);

    EXPECT_TRUE(vectors_are_equal(result, expected_result));
}

// Test case: 90-degree rotation around the x-axis
TEST(CommonMathTest, Rotate90DegreesXAxis) {
    matrix3d_t rotation_matrix = {
        1.0f,
        0.0f,
        0.0f, // 90-degree rotation around the x-axis
        0.0f,
        0.0f,
        -1.0f,
        0.0f,
        1.0f,
        0.0f
    };

    vector3d_t input_vector = {0.0f, 1.0f, 0.0f}; // Vector along y-axis
    vector3d_t expected_result = {
        0.0f, 0.0f, 1.0f
    }; // Expected result after 90-degree rotation around x-axis

    vector3d_t result = math_vector3d_rotate(&rotation_matrix, &input_vector);

    EXPECT_TRUE(vectors_are_equal(result, expected_result));
}

// Test case: 90-degree rotation around the z-axis
TEST(CommonMathTest, Rotate90DegreesZAxis) {
    matrix3d_t rotation_matrix = {
        0.0f,
        -1.0f,
        0.0f, // 90-degree rotation around the z-axis
        1.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        1.0f
    };

    vector3d_t input_vector = {1.0f, 0.0f, 0.0f}; // Vector along x-axis
    vector3d_t expected_result = {
        0.0f, 1.0f, 0.0f
    }; // Expected result after 90-degree rotation around z-axis

    vector3d_t result = math_vector3d_rotate(&rotation_matrix, &input_vector);

    EXPECT_TRUE(vectors_are_equal(result, expected_result));
}
