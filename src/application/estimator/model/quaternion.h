#ifndef QUATERNION_H
#define QUATERNION_H

#include "application/estimator/estimator.h"
#include "common/math/math.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Function to compute the norm of a quaternion
double quaternion_norm(quaternion_t q);

// Function to normalize a quaternion
quaternion_t quaternion_normalize(quaternion_t q);

// quaternion_t multiplication
quaternion_t quaternion_multiply(quaternion_t q1, quaternion_t q2);

// Compute rotation matrix from a quaternion
void quaternion_rotmatrix(quaternion_t q, double S[3][3]);

// Rotate a vector using a quaternion
vector3d_t quaternion_rotate(quaternion_t q, vector3d_t v);

// Quaternion derivative
quaternion_t quaternion_derivative(quaternion_t q, vector3d_t w);

#endif