#include "fff.h"
#include <gtest/gtest.h>

extern "C" {
// add includes like freertos, hal, proc headers, etc
#include "application/estimator/estimator_types.h"
#include "application/estimator/model/quaternion.h"
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include <math.h>
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

// Unit test for vector rotation
TEST(CommonMathTest, NonTrivialRotation) {
    // Define a non-trivial rotation matrix and vector
    matrix3d_t rotation_matrix = {
        .s11 = 0.5f,
        .s12 = 0.5f,
        .s13 = -0.707f,
        .s21 = -0.707f,
        .s22 = 0.707f,
        .s23 = 0.0f,
        .s31 = 0.5f,
        .s32 = 0.5f,
        .s33 = 0.707f
    };
    vector3d_t vector = {1.0f, 2.0f, 3.0f};

    // Expected result from MATLAB
    vector3d_t expected_result = {-0.6210, 0.7070, 3.6210};

    // Perform the matrix-vector multiplication
    vector3d_t actual_result = math_vector3d_rotate(&rotation_matrix, &vector);

    // Define tolerance for floating point comparison
    float tolerance = 1e-3f;

    // Check if the computed result is close to the expected result
    EXPECT_NEAR(actual_result.x, expected_result.x, tolerance);
    EXPECT_NEAR(actual_result.y, expected_result.y, tolerance);
    EXPECT_NEAR(actual_result.z, expected_result.z, tolerance);
}


// vector norm
TEST(CommonMathTest, VectorNormTest) {
    // sample values
    vector3d_t sample_input[4] = {
        {.array = {56.782164072522114, 7.585428956306361, 5.395011866660715}},
        {.array = {53.079755300897268, 77.916723010201110, 93.401068422918300}},
        {.array = {12.990620847373012, 56.882366087219275, 46.939064105820584}},
        {.array = {1.190206950124140, 33.712264439888152, 16.218230819324276}}
    };

    // Expected result from MATLAB
    double expected_output[4] = {
        57.540064670205254, 1.327110987445820e+02, 74.884147459999653, 37.429458681070329
    };

    for (int i = 0; i < 4; i++) {
        double result = math_vector3d_norm(&sample_input[i]);
        EXPECT_NEAR(result, expected_output[i], 1e-5);
    }
}

/**
 * math.h helper functions
 */
// Unit test for cotangent function
TEST(CommonMathTest, CotangentTest) {
    // sample values
    double sample_input[10] = {
        0.905791937075619,
        0.126986816293506,
        0.913375856139019,
        0.632359246225410,
        0.097540404999410,
        0.278498218867048,
        0.546881519204984,
        0.957506835434298,
        0.964888535199277,
        0.157613081677548
    };

    // Expected result from MATLAB
    double expected_result[10] = {
        0.784154973927230,
        7.832458710118800,
        0.771979881900071,
        1.364750986561514,
        10.219627546002505,
        3.497370908906658,
        1.642514379845794,
        0.703826530158896,
        0.692844998992024,
        6.292026114333982
    };

    for (int i = 0; i < 10; i++) {
        EXPECT_NEAR(cot(sample_input[i]), expected_result[i], 1e-6);
    }
}

