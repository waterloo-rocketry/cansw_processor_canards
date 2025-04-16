/**
 * Quaternion math operations
 */
#include "application/estimator/model/quaternion.h"
#include "common/math/math.h"
#include <math.h>

// Norm of a quaternion
double quaternion_norm(const quaternion_t *q) {
    return sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

// Normalize quaternion
quaternion_t quaternion_normalize(const quaternion_t *q) {
    double norm = quaternion_norm(q);

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

// Jacobian of the rotation wrt to the quaternion 
// Rotation: vector3d_t rotated = math_vector3d_rotate(quaternion_rotmatrix(quaternion_t), vector3d_t)
// output is an array with 3 rows, 4 cols
void quaternion_rotate_jacobian(double R_q[3][4], const quaternion_t *q_unnormed, const vector3d_t *v) {

    quaternion_t q = quaternion_normalize(q_unnormed);

    // R_q = 2 * [qw*v - qv x v, qv'*v*I + qv*v' - v*qv' + qw*v_tilde]
    R_q[0][0] = 2 * ( q.w * v->x - q.y * v->z + q.z * v->y );
    R_q[0][1] = 2 * ( q.x * v->x + q.y * v->y + q.z * v->z );
    R_q[0][2] = 2 * ( q.x * v->y - q.w * v->z - q.y * v->x );
    R_q[0][3] = 2 * ( q.w * v->y + q.x * v->z - q.z * v->x );

    R_q[1][0] = 2 * ( q.w * v->y + q.x * v->z - q.z * v->x );
    R_q[1][1] = 2 * ( q.w * v->z - q.x * v->y + q.y * v->x );
    R_q[1][2] = 2 * ( q.x * v->x + q.y * v->y + q.z * v->z );
    R_q[1][3] = 2 * ( q.y * v->z - q.w * v->x - q.z * v->y );

    R_q[2][0] = 2 * ( q.w * v->z - q.x * v->y + q.y * v->x );
    R_q[2][1] = 2 * ( q.z * v->x - q.x * v->z - q.w * v->y );
    R_q[2][2] = 2 * ( q.w * v->x - q.y * v->z + q.z * v->y );
    R_q[2][3] = 2 * ( q.x * v->x + q.y * v->y + q.z * v->z );

    return;
}


// Quaternion time derivative
quaternion_t quaternion_derivative(const quaternion_t *q, const vector3d_t *omega) {
    quaternion_t q_normed = quaternion_normalize(q);

    quaternion_t omega_q = {.array = {0, 0.5 * omega->x, 0.5 * omega->y, 0.5 * omega->z}};

    quaternion_t q_dot = quaternion_multiply(&q_normed, &omega_q);

    return q_dot;
}

// Jacobian of the derivative wrt to the quaternion qdot_q, and wrt to the rates qdot_w
// quaternion_t qdot = quaternion_derivative(quaternion_t *q, vector3d_t *w);
// Output are arrays with: qdot_q 4 rows and 4 cols, qdot_w 4 rows and 3 cols
void quaternion_derivative_jacobian(double qdot_q[4][4], double qdot_w[4][3], const quaternion_t *q, const vector3d_t *w) {
    
    // qdot partial q (4x4)
    qdot_q[0][0] = 0;           qdot_q[0][1] = -0.5 * w->x;  qdot_q[0][2] = -0.5 * w->y;  qdot_q[0][3] = -0.5 * w->z;
    qdot_q[1][0] = 0.5 * w->x;  qdot_q[1][1] =  0;           qdot_q[1][2] =  0.5 * w->z;  qdot_q[1][3] = -0.5 * w->y;
    qdot_q[2][0] = 0.5 * w->y;  qdot_q[2][1] = -0.5 * w->z;  qdot_q[2][2] =  0;           qdot_q[2][3] =  0.5 * w->x;
    qdot_q[3][0] = 0.5 * w->z;  qdot_q[3][1] =  0.5 * w->y;  qdot_q[3][2] = -0.5 * w->x;  qdot_q[3][3] =  0;

    // qdot partial rates (4x3)
    qdot_w[0][0] = -0.5 * q->x;  qdot_w[0][1] = -0.5 * q->y;  qdot_w[0][2] = -0.5 * q->z;
    qdot_w[1][0] =  0.5 * q->w;  qdot_w[1][1] = -0.5 * q->z;  qdot_w[1][2] =  0.5 * q->y;
    qdot_w[2][0] =  0.5 * q->z;  qdot_w[2][1] =  0.5 * q->w;  qdot_w[2][2] = -0.5 * q->x;
    qdot_w[3][0] = -0.5 * q->y;  qdot_w[3][1] =  0.5 * q->x;  qdot_w[3][2] =  0.5 * q->w;
    
    return;
}

// !! this is possibly incorrect !!
// // Approximate solution of quaternion differential equation (truncation of Taylor expansion)
// quaternion_t quaternion_increment(const quaternion_t *q, const vector3d_t *omega, double deltaT)
// {
//     quaternion_t q_normed = quaternion_normalize(q);

//     quaternion_t omega_q = {.array = {0, omega->x, omega->y, omega->z}};
//     double omega_norm = quaternion_norm(&omega_q);

//     // incremental quaternion difference
//     double dphi = 0.5 * omega_norm * deltaT;
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
double quaternion_to_roll(const quaternion_t *q) {
    double roll;
    roll = atan2(
        2 * (q->y * q->z + q->w * q->x), (q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z)
    );
    return roll;
}