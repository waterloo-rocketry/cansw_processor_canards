/**
 * Special operations in R3, as they are used often
 * Cross product only exists in R3
 * Rotation matrix transpose is inverse rotation
 */
#include "common/math/math-algebra3d.h"
#include "common/math/math.h"
#include <math.h>

// vector * scalar // vector scaling
vector3d_t math_vector3d_scale(float scalar, const vector3d_t *vector) {
    vector3d_t result;
    result.x = scalar * vector->x;
    result.y = scalar * vector->y;
    result.z = scalar * vector->z;
    return result;
}

// ||vector|| // norm of a vector
double math_vector3d_norm(const vector3d_t *vector) {
    double norm = sqrt((vector->x * vector->x) + (vector->y * vector->y) + (vector->z * vector->z));
    return norm;
}

// vector + vector // vector addition
vector3d_t math_vector3d_add(const vector3d_t *vector1, const vector3d_t *vector2) {
    vector3d_t result;
    result.x = vector1->x + vector2->x;
    result.y = vector1->y + vector2->y;
    result.z = vector1->z + vector2->z;
    return result;
}

// vector - vector // vector subtraction
vector3d_t math_vector3d_subt(const vector3d_t *vector_left, const vector3d_t *vector_right) {
    vector3d_t result;
    result.x = vector_left->x - vector_right->x;
    result.y = vector_left->y - vector_right->y;
    result.z = vector_left->z - vector_right->z;
    return result;
}

// vector * vector // vector dot product
float math_vector3d_dot(const vector3d_t *vector1, const vector3d_t *vector2) {
    float result;
    result = (vector1->x * vector2->x) + (vector1->y * vector2->y) + (vector1->z * vector2->z);
    return result;
}

// vector x vector // vector cross product, only in R3
vector3d_t math_vector3d_cross(const vector3d_t *vector_left, const vector3d_t *vector_right) {
    vector3d_t result;
    result.x = vector_left->y * vector_right->z - vector_left->z * vector_right->y;
    result.y = vector_left->z * vector_right->x - vector_left->x * vector_right->z;
    result.z = vector_left->x * vector_right->y - vector_left->y * vector_right->x;
    return result;
}
#include <stdio.h>
// matrix * vector // matrix vector multiplication, for rotating vectors
vector3d_t math_vector3d_rotate(const matrix3d_t *matrix, const vector3d_t *vector) {
    vector3d_t result;
    result.x = matrix->s11 * vector->x + matrix->s12 * vector->y + matrix->s13 * vector->z;
    result.y = matrix->s21 * vector->x + matrix->s22 * vector->y + matrix->s23 * vector->z;
    result.z = matrix->s31 * vector->x + matrix->s32 * vector->y + matrix->s33 * vector->z;
    return result;
}

// matrix ^T // transpose, for inverting rotation matrices
matrix3d_t math_matrix3d_transp(const matrix3d_t *input) {
    matrix3d_t result;
    result.s11 = input->s11;
    result.s12 = input->s21;
    result.s13 = input->s31;
    result.s21 = input->s12;
    result.s22 = input->s22;
    result.s23 = input->s32;
    result.s31 = input->s13;
    result.s32 = input->s23;
    result.s33 = input->s33;
    return result;
}
