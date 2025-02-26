#ifndef QUATERNION_H
#define QUATERNION_H

#include "application/estimator/estimator.h"
#include "common/math/math.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// // Norm of a quaternion
float quaternion_norm(quaternion_t q);

// Normalize quaternion
quaternion_t quaternion_normalize(quaternion_t q);

// Quaternion multiplication
quaternion_t quaternion_multiply(quaternion_t q1, quaternion_t q2);

// Compute rotation matrix from a quaternion
matrix3d_t quaternion_rotmatrix(quaternion_t q);

// Quaternion time derivative
quaternion_t quaternion_derivative(quaternion_t q, vector3d_t omega);

// Discrete update of quaternion differential equation
quaternion_t quaternion_increment(quaternion_t q, vector3d_t omega, float deltaT);

// Compute Euler angles from a quaternion
vector3d_t quaternion_to_euler(quaternion_t q);

#endif