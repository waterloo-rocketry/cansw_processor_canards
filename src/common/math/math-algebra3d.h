#ifndef COMMON_MATH_ALGEBRA3D_H
#define COMMON_MATH_ALGEBRA3D_H

#include "common/math/math.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * Special operations in R3, as they are used often
 * Cross product only exists in R3 
 * Rotation matrix transpose is inverse rotation
 */
// vector * scalar | vector scaling
vector3d_t math_vector_scale(float scalar, vector3d_t *vector);

// vector + vector | vector addition
vector3d_t math_vector_add(const vector3d_t *vector1, const vector3d_t *vector2);

// vector - vector | vector subtraction
vector3d_t math_vector_subt(const vector3d_t *vector_left, const vector3d_t *vector_right);

// vector * vector | vector dot product
float math_vector_dot(const vector3d_t *vector1, const vector3d_t *vector2);

// vector x vector | vector cross product, only in R3
vector3d_t math_vector_cross(const vector3d_t *vector_left, const vector3d_t *vector_right);

// matrix * vector | matrix vector multiplication, for rotating vectors
vector3d_t math_vector_rotate(const matrix3d_t *matrix, const vector3d_t *vector_right);

// matrix ^T | transpose, for inverting rotation matrices
matrix3d_t math_matrix_transp3(const matrix3d_t *input);

#endif