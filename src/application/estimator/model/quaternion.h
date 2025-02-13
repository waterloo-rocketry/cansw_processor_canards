#ifndef QUATERNION_H
#define QUATERNION_H

#include "application/estimator/estimator.h"
#include "common/math/math.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Function to compute the norm of a quaternion
float quaternion_norm(quaternion_t q);

// Function to normalize a quaternion
quaternion_t quaternion_normalize(quaternion_t q);

// quaternion_t multiplication
quaternion_t quaternion_multiply(quaternion_t q1, quaternion_t q2);

// Compute rotation matrix from a quaternion
matrix3d_t quaternion_rotmatrix(quaternion_t q);

// Quaternion time derivative
quaternion_t quaternion_derivative(quaternion_t q, vector3d_t omega);

// Discrete update of quaternion differential equation
quaternion_t quaternion_increment(quaternion_t q, vector3d_t omega, float deltaT);

#endif