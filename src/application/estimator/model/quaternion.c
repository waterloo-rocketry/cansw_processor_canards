/**
 * Quaternion math operations
 */
#include "application/estimator/model/quaternion.h"
#include "common/math/math.h"
#include <math.h>

// Norm of a quaternion
float quaternion_norm(const quaternion_t *q) {
    return sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

// Normalize quaternion
quaternion_t quaternion_normalize(const quaternion_t *q) {
    float norm = quaternion_norm(q);

    quaternion_t result = {.array = {q->w / norm, q->x / norm, q->y / norm, q->z / norm}};

    return result;
}

// Quaternion multiplication
quaternion_t quaternion_multiply(const quaternion_t *q1, const quaternion_t *q2) {
    quaternion_t result;

    result.w = q2->w * q1->w - q2->x * q1->x - q2->y * q1->y - q2->z * q1->z;
    result.x = q2->w * q1->x + q2->x * q1->w - q2->y * q1->z + q2->z * q1->y;
    result.y = q2->w * q1->y + q2->x * q1->z + q2->y * q1->w - q2->z * q1->x;
    result.z = q2->w * q1->z - q2->x * q1->y + q2->y * q1->x + q2->z * q1->w;

    return result;
}

// Rotation matrix from quaternion: point rotation/active transformation
matrix3d_t quaternion_rotmatrix(const quaternion_t *q_unnormed) {
    quaternion_t q = quaternion_normalize(q_unnormed);

    matrix3d_t S;

    // top row
    S.s11 = 1 - 2 * (q.y * q.y + q.z * q.z);
    S.s12 = 2 * (q.x * q.y + q.w * q.z);
    S.s13 = 2 * (q.x * q.z - q.w * q.y);
    // middle row
    S.s21 = 2 * (q.x * q.y - q.w * q.z);
    S.s22 = 1 - 2 * (q.x * q.x + q.z * q.z);
    S.s23 = 2 * (q.y * q.z + q.w * q.x);
    // bottom row
    S.s31 = 2 * (q.x * q.z + q.w * q.y);
    S.s32 = 2 * (q.y * q.z - q.w * q.x);
    S.s33 = 1 - 2 * (q.x * q.x + q.y * q.y);

    return S;
}

// Quaternion time derivative
quaternion_t quaternion_derivative(const quaternion_t *q, const vector3d_t *omega) {
    quaternion_t q_normed = quaternion_normalize(q);

    quaternion_t omega_q = {.array = {0, 0.5 * omega->x, 0.5 * omega->y, 0.5 * omega->z}};

    quaternion_t q_dot = quaternion_multiply(&q_normed, &omega_q);

    return q_dot;
}

// !! this is possibly incorrect !!
// // Approximate solution of quaternion differential equation (truncation of Taylor expansion)
// quaternion_t quaternion_increment(const quaternion_t *q, const vector3d_t *omega, float deltaT) {
//     quaternion_t q_normed = quaternion_normalize(q);

//     quaternion_t omega_q = {.array = {0, omega->x, omega->y, omega->z}};
//     float omega_norm = quaternion_norm(&omega_q);

//     // incremental quaternion difference
//     float dphi = 0.5 * omega_norm * deltaT;
//     quaternion_t dq = quaternion_normalize(&omega_q);
//     dq.w = cos(dphi);
//     dq.x = omega_q.x * sin(dphi);
//     dq.y = omega_q.y * sin(dphi);
//     dq.z = omega_q.z * sin(dphi);

//     // update quaternion attitude
//     quaternion_t q_new = quaternion_multiply(&q_normed, &dq);

//     return q_new;
// }

// Compute Euler angles from a quaternion
vector3d_t quaternion_to_euler(const quaternion_t *q) {
    vector3d_t euler = {.array = {0, 0, 0}};

    // yaw angle
    euler.z = atan2(
        2 * (q->x * q->y + q->w * q->z), (q->w * q->w + q->x * q->x - q->y * q->y - q->z * q->z)
    );
    // pitch angle
    euler.y = asin(-2 * (q->x * q->z - q->w * q->y));
    // roll angle
    euler.x = atan2(
        2 * (q->y * q->z + q->w * q->x), (q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z)
    );

    return euler;
}

// Compute Euler angle roll from a quaternion
float quaternion_to_roll(const quaternion_t *q) {
    float roll;
    roll = atan2(
        2 * (q->y * q->z + q->w * q->x), (q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z)
    );
    return roll;
}
