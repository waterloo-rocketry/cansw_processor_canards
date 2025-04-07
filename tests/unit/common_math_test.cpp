#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/quaternion.h"
#include "application/estimator/pad_filter.h"
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

// Test case: Rotation matrix from quaternion for a known quaternion
TEST(CommonMathTest, KnownQuaternionTest) {
    quaternion_t q = {0.707f, 0.0f, 0.707f, 0.0f}; // A 90-degree rotation around the y-axis

    matrix3d_t expected_matrix = {
        0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f
    }; // Expected rotation matrix

    matrix3d_t result_matrix = quaternion_rotmatrix(&q);

    EXPECT_TRUE(matrices_are_equal(result_matrix, expected_matrix));
}

// Test case: Identity quaternion (no rotation)
TEST(CommonMathTest, IdentityQuaternionTest) {
    quaternion_t identity_q = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity quaternion

    matrix3d_t expected_matrix = {
        1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f
    }; // Identity matrix

    matrix3d_t result_matrix = quaternion_rotmatrix(&identity_q);

    EXPECT_TRUE(matrices_are_equal(result_matrix, expected_matrix));
}

// Test case: Zero quaternion (should result in identity matrix, though it's invalid)
TEST(CommonMathTest, ZeroQuaternionTest) {
    quaternion_t zero_q = {0.0f, 0.0f, 0.0f, 0.0f}; // Zero quaternion

    // This test case should throw an exception or handle the error gracefully, but for now let's
    // assume it defaults to identity matrix
    matrix3d_t expected_matrix = {
        1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f
    }; // Identity matrix

    matrix3d_t result_matrix = quaternion_rotmatrix(&zero_q);

    EXPECT_TRUE(matrices_are_equal(result_matrix, expected_matrix));
}

// Test case: 180-degree rotation around the z-axis (known quaternion)
TEST(CommonMathTest, Rotate180DegreesZAxis) {
    quaternion_t q = {0.0f, 0.0f, 0.707f, 0.707f}; // 180-degree rotation around z-axis

    matrix3d_t expected_matrix = {
        -1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f
    }; // Expected rotation matrix for 180-degree rotation

    matrix3d_t result_matrix = quaternion_rotmatrix(&q);

    EXPECT_TRUE(matrices_are_equal(result_matrix, expected_matrix));
}

// Test case: Quaternion normalization (should not affect the result)
TEST(CommonMathTest, NormalizationTest) {
    quaternion_t q = {2.0f, 0.0f, 2.0f, 0.0f}; // A non-normalized quaternion

    quaternion_t normalized_q = quaternion_normalize(&q); // Normalize quaternion
    matrix3d_t result_matrix = quaternion_rotmatrix(&normalized_q);

    quaternion_t expected_normalized_q = {
        0.707f, 0.0f, 0.707f, 0.0f
    }; // Normalized quaternion (same as before)

    matrix3d_t expected_matrix = {
        0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f
    }; // Expected rotation matrix

    EXPECT_TRUE(matrices_are_equal(result_matrix, expected_matrix));
}
