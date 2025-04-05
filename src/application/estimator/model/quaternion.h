/**
 * Quaternion math operations
 */
#ifndef QUATERNION_H
#define QUATERNION_H

#include "common/math/math.h"
#include <math.h>

// // Norm of a quaternion
float quaternion_norm(const quaternion_t *q);

// Normalize quaternion
quaternion_t quaternion_normalize(const quaternion_t *q);

// Quaternion multiplication
quaternion_t quaternion_multiply(const quaternion_t *q1, const quaternion_t *q2);

// Compute rotation matrix from a quaternion
matrix3d_t quaternion_rotmatrix(const quaternion_t *q);

// Quaternion time derivative
quaternion_t quaternion_derivative(const quaternion_t *q, const vector3d_t *omega);

// Discrete update of quaternion differential equation
quaternion_t quaternion_increment(const quaternion_t *q, const vector3d_t *omega, float deltaT);

// Compute Euler angles from a quaternion
vector3d_t quaternion_to_euler(const quaternion_t *q);

// Compute Euler angle roll from a quaternion
float quaternion_to_roll(const quaternion_t *q);

#endif