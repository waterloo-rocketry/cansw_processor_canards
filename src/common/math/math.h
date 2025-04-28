/**
 * Math types and utils used in the processor project
 */
#ifndef COMMON_MATH_H
#define COMMON_MATH_H

#include <math.h>

#define SIZE_VECTOR_3D 3
#define SIZE_QUAT 4
#define SIDE_MATRIX_3D 3

/**
 * 3D vector.
 */
typedef union {
    double array[SIZE_VECTOR_3D];
    struct {
        double x;
        double y;
        double z;
    };
} vector3d_t;

/**
 * Quaternion.
 */
typedef union {
    double array[SIZE_QUAT];
    struct {
        double w;
        double x;
        double y;
        double z;
    };
} quaternion_t;

/**
 * 3D (rotation) matrix.
 */
typedef union {
    double array[SIDE_MATRIX_3D][SIDE_MATRIX_3D];

    // elements sij, with the row i and the column j
    struct {
        double s11, s12, s13;
        double s21, s22, s23;
        double s31, s32, s33;
    };

    double flat[SIDE_MATRIX_3D * SIDE_MATRIX_3D];
} matrix3d_t;

// helper function for estimator models
static inline double cot(double x) {
    return 1 / tan(x);
}

#endif // COMMON_MATH_H