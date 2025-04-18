/**
 * Math types and utils used in the processor project
 */
#ifndef COMMON_MATH_H
#define COMMON_MATH_H

/**
 * 3D vector.
 */
typedef union {
    double array[3];
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
    double array[4];
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
    double array[3][3];
    // elements sij, with the row i and the column j
    struct {
        double s11, s12, s13;
        double s21, s22, s23;
        double s31, s32, s33;
    };
} matrix3d_t;

#endif // COMMON_MATH_H