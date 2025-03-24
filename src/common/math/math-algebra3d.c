#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/**
 * Special operations in R3, as they are used often
 * Cross product only exists in R3 
 * Rotation matrix transpose is inverse rotation
 */
// vector * scalar | vector scaling
vector3d_t math_vector_scale(float scalar, vector3d_t vector)
{
    vector3d_t result;
    result.x = scalar * vector.x;
    result.y = scalar * vector.y;
    result.z = scalar * vector.z;
    return result;
}

// vector + vector | vector addition
vector3d_t math_vector_add(vector3d_t vector1, vector3d_t vector2)
{
    vector3d_t result;
    result.x = vector1.x + vector2.x;
    result.y = vector1.y + vector2.y;
    result.z = vector1.z + vector2.z;
    return result;
}

// vector * vector | vector dot product
float math_vector_dot(vector3d_t vector1, vector3d_t vector2)
{
    float result;
    result = (vector1.x * vector2.x) + (vector1.y * vector2.y) + (vector1.z * vector2.z);
    return result;
}

// vector x vector | vector cross product, only in R3
vector3d_t math_vector_cross(vector3d_t vector_left, vector3d_t vector_right);

// matrix * vector | matrix vector multiplication, for rotating vectors
vector3d_t math_vector_rotate(matrix3d_t *matrix, vector3d_t vector_right);

// matrix ^T | transpose, for inverting rotation matrices
matrix3d_t math_matrix_transp3(const matrix3d_t *input);
