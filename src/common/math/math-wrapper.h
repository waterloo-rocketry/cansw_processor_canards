#ifndef COMMON_MATH_WRAPPER_H
#define COMMON_MATH_WRAPPER_H

#include "common/math/math.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * Special operations
 * Cross product only exists in R3 
 * Rotation matrix transpose is used often
 */
// vector x vector | vector cross product, only in R3
vector3d_t math_vector_cross(vector3d_t vector_left, vector3d_t vector_right);

// matrix * vector | matrix vector multiplication, only in R3 for rotating vectors
vector3d_t math_vector_rotate(matrix3d_t matrix, vector3d_t vector_right);

// matrix ^T | transpose, only in R3x3 for inverting rotation matrices
matrix3d_t math_matrix_transp3(matrix3d_t input);

/**
 * General operations
 */
// vector * scalar | vector scaling
void math_vector_scale(float *result, float *vector, float scalar);

// vector + vector | vector addition
void math_vector_add(float *result, float *vector_left, float *vector_right);

// vector - vector | vector subtraction
void math_vector_subt(float *result, float *vector_left, float *vector_right);

// matrix ^T | transpose
void math_matrix_transp(float *result, float *input);

// matrix ^-1| inverse
void math_matrix_invert(float *result, float *input);

// matrix + matrix | matrix addition
void math_matrix_add(float *result, float *matrix_left, float *matrix_right);

// matrix - matrix | matrix subtraction
void math_matrix_subt(float *result, float *matrix_left, float *matrix_right);

// matrix * vector | matrix vector multiplication
void math_matrix_vector(float *result, float *matrix, float *vector_right);

// matrix * matrix | matrix matrix multiplication
void math_matrix_matrix(float *result, float *matrix_left, float *matrix_right);

/**
 * Creations
 */
// matrix diagonal | distribute vector on main diagonal of matrix
void math_matrix_diag(float *result, float *matrix, float *vector);

// matrix identity | create identity matrix of specific size
void math_matrix_identity(float *result, int size);


#endif