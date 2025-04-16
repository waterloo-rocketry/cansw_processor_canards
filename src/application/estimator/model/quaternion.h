/**
 * Quaternion math operations
 */
#ifndef QUATERNION_H
#define QUATERNION_H

#include "common/math/math.h"

// // Norm of a quaternion
double quaternion_norm(const quaternion_t *q);

// Normalize quaternion
quaternion_t quaternion_normalize(const quaternion_t *q);

// Invert attitude quaternion
quaternion_t quaternion_invert(const quaternion_t *q);

// Quaternion multiplication
quaternion_t quaternion_multiply(const quaternion_t *q1, const quaternion_t *q2);

// Compute rotation matrix from a quaternion
matrix3d_t quaternion_rotmatrix(const quaternion_t *q);

// Jacobian of the rotation wrt to the quaternion 
// Rotation: vector3d_t rotated = math_vector3d_rotate(quaternion_rotmatrix(quaternion_t), vector3d_t)
// output is an array with 3 rows, 4 cols
void quaternion_rotate_jacobian(double R_q[3][4], const quaternion_t *q, const vector3d_t *vector);

// Quaternion time derivative
quaternion_t quaternion_derivative(const quaternion_t *q, const vector3d_t *rates);

// Jacobian of the derivative wrt to the quaternion qdot_q, and wrt to the rates qdot_w
// quaternion_t qdot = quaternion_derivative(quaternion_t *q, vector3d_t *w);
// Output are arrays with: qdot_q 4 rows and 4 cols, qdot_w 4 rows and 3 cols
void quaternion_derivative_jacobian(double qdot_q[4][4], double qdot_w[4][3], const quaternion_t *q, const vector3d_t *vector);

// // Discrete update of quaternion differential equation
// quaternion_t quaternion_increment(const quaternion_t *q, const vector3d_t *omega, double deltaT);

// Compute Euler angles from a quaternion
vector3d_t quaternion_to_euler(const quaternion_t *q);

double quaternion_to_roll(const quaternion_t *q);

#endif