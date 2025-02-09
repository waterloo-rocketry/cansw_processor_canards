#include "application/estimator/model/quaternion.h"
#include "application/estimator/estimator.h"
#include "common/math/math.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Function to compute the norm of a quaternion
double quaternion_norm(quaternion_t q)
{
    return sqrt(q.element.w * q.element.w + q.element.x * q.element.x + q.element.y * q.element.y + q.element.z * q.element.z);
}

// Function to normalize a quaternion
quaternion_t quaternion_normalize(quaternion_t q)
{
    double norm = quaternion_norm(q);

    quaternion_t result = {.array = {q.element.w / norm, q.element.x / norm, q.element.y / norm, q.element.z / norm}};

    return result;
}

// quaternion_t multiplication
quaternion_t quaternion_multiply(quaternion_t q1, quaternion_t q2)
{
    quaternion_t result;

    result.element.w = q1.element.w * q2.element.w - q1.element.x * q2.element.x - q1.element.y * q2.element.y - q1.element.z * q2.element.z;
    result.element.x = q1.element.w * q2.element.x + q2.element.w * q1.element.x + (q1.element.y * q2.element.z - q1.element.z * q2.element.y);
    result.element.y = q1.element.w * q2.element.y + q2.element.w * q1.element.y + (q1.element.z * q2.element.x - q1.element.x * q2.element.z);
    result.element.z = q1.element.w * q2.element.z + q2.element.w * q1.element.z + (q1.element.x * q2.element.y - q1.element.y * q2.element.x);

    return result;
}

// Compute rotation matrix from a quaternion
void quaternion_rotmatrix(quaternion_t q, double S[3][3])
{
    q = quaternion_normalize(q);

    S[0][0] = 1 - 2 * (q.element.y * q.element.y + q.element.z * q.element.z);
    S[0][1] = 2 * (q.element.x * q.element.y - q.element.w * q.element.z);
    S[0][2] = 2 * (q.element.x * q.element.z + q.element.w * q.element.y);
    S[1][0] = 2 * (q.element.x * q.element.y + q.element.w * q.element.z);
    S[1][1] = 1 - 2 * (q.element.x * q.element.x + q.element.z * q.element.z);
    S[1][2] = 2 * (q.element.y * q.element.z - q.element.w * q.element.x);
    S[2][0] = 2 * (q.element.x * q.element.z - q.element.w * q.element.y);
    S[2][1] = 2 * (q.element.y * q.element.z + q.element.w * q.element.x);
    S[2][2] = 1 - 2 * (q.element.x * q.element.x + q.element.y * q.element.y);
}

// Rotate a vector using a quaternion
vector3d_t quaternion_rotate(quaternion_t q, vector3d_t v)
{
    q = quaternion_normalize(q);

    quaternion_t v_quat = {0, v.component.x, v.component.y, v.component.z};
    quaternion_t q_conjugate = {q.element.w, -q.element.x, -q.element.y, -q.element.z};
    quaternion_t qv = quaternion_multiply(q, v_quat);
    quaternion_t rotated = quaternion_multiply(qv, q_conjugate);
    vector3d_t result = {.array = {rotated.element.x, rotated.element.y, rotated.element.z}};

    return result;
}

// Quaternion derivative
quaternion_t quaternion_derivative(quaternion_t q, vector3d_t omega)
{
    quaternion_t q_dot;
    q = quaternion_normalize(q);

    double W_matrix[4][4] = {
        {0, -omega.component.x, -omega.component.y, -omega.component.z},
        {omega.component.x, 0, omega.component.z, -omega.component.y},
        {omega.component.y, -omega.component.z, 0, omega.component.x},
        {omega.component.z, omega.component.y, -omega.component.x, 0}};

    q_dot.element.w = 0.5 * (W_matrix[0][0] * q.element.w + W_matrix[0][1] * q.element.x + W_matrix[0][2] * q.element.y + W_matrix[0][3] * q.element.z);
    q_dot.element.x = 0.5 * (W_matrix[1][0] * q.element.w + W_matrix[1][1] * q.element.x + W_matrix[1][2] * q.element.y + W_matrix[1][3] * q.element.z);
    q_dot.element.y = 0.5 * (W_matrix[2][0] * q.element.w + W_matrix[2][1] * q.element.x + W_matrix[2][2] * q.element.y + W_matrix[2][3] * q.element.z);
    q_dot.element.z = 0.5 * (W_matrix[3][0] * q.element.w + W_matrix[3][1] * q.element.x + W_matrix[3][2] * q.element.y + W_matrix[3][3] * q.element.z);

    return q_dot;
}
