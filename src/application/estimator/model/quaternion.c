#include "application/estimator/model/quaternion.h"
#include "application/estimator/estimator.h"
#include "common/math/math.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Function to compute the norm of a quaternion
double quaternion_norm(quaternion_t q)
{
    return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

// Function to normalize a quaternion
quaternion_t quaternion_normalize(quaternion_t q)
{
    double norm = quaternion_norm(q);

    quaternion_t result = {.array = {q.w / norm, q.x / norm, q.y / norm, q.z / norm}};

    return result;
}

// quaternion_t multiplication
quaternion_t quaternion_multiply(quaternion_t q1, quaternion_t q2)
{
    quaternion_t result;

    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q2.w * q1.x + (q1.y * q2.z - q1.z * q2.y);
    result.y = q1.w * q2.y + q2.w * q1.y + (q1.z * q2.x - q1.x * q2.z);
    result.z = q1.w * q2.z + q2.w * q1.z + (q1.x * q2.y - q1.y * q2.x);

    return result;
}

// Compute rotation matrix from a quaternion
void quaternion_rotmatrix(quaternion_t q, double S[3][3])
{
    q = quaternion_normalize(q);

    S[0][0] = 1 - 2 * (q.y * q.y + q.z * q.z);
    S[0][1] = 2 * (q.x * q.y - q.w * q.z);
    S[0][2] = 2 * (q.x * q.z + q.w * q.y);
    S[1][0] = 2 * (q.x * q.y + q.w * q.z);
    S[1][1] = 1 - 2 * (q.x * q.x + q.z * q.z);
    S[1][2] = 2 * (q.y * q.z - q.w * q.x);
    S[2][0] = 2 * (q.x * q.z - q.w * q.y);
    S[2][1] = 2 * (q.y * q.z + q.w * q.x);
    S[2][2] = 1 - 2 * (q.x * q.x + q.y * q.y);
}

// Rotate a vector using a quaternion
vector3d_t quaternion_rotate(quaternion_t q, vector3d_t v)
{
    q = quaternion_normalize(q);

    quaternion_t v_quat = {0, v.x, v.y, v.z};
    quaternion_t q_conjugate = {q.w, -q.x, -q.y, -q.z};
    quaternion_t qv = quaternion_multiply(q, v_quat);
    quaternion_t rotated = quaternion_multiply(qv, q_conjugate);
    vector3d_t result = {.array = {rotated.x, rotated.y, rotated.z}};

    return result;
}

// Quaternion derivative
quaternion_t quaternion_derivative(quaternion_t q, vector3d_t omega)
{
    quaternion_t q_dot;
    q = quaternion_normalize(q);

    double W[4][4] = {
        {0, -omega.x, -omega.y, -omega.z},
        {omega.x, 0, omega.z, -omega.y},
        {omega.y, -omega.z, 0, omega.x},
        {omega.z, omega.y, -omega.x, 0}};

    q_dot.w = 0.5 * (W[0][0] * q.w + W[0][1] * q.x + W[0][2] * q.y + W[0][3] * q.z);
    q_dot.x = 0.5 * (W[1][0] * q.w + W[1][1] * q.x + W[1][2] * q.y + W[1][3] * q.z);
    q_dot.y = 0.5 * (W[2][0] * q.w + W[2][1] * q.x + W[2][2] * q.y + W[2][3] * q.z);
    q_dot.z = 0.5 * (W[3][0] * q.w + W[3][1] * q.x + W[3][2] * q.y + W[3][3] * q.z);

    return q_dot;
}
