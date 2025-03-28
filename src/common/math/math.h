/**
 * Math types and utils used in the processor project
 */
#ifndef COMMON_MATH_H
#define COMMON_MATH_H

/**
 * 3D vector.
 */
typedef union {
    float array[3];
    struct {
        float x;
        float y;
        float z;
    };
} vector3d_t;

/**
 * Quaternion.
 */
typedef union {
    float array[4];
    struct {
        float w;
        float x;
        float y;
        float z;
    };
} quaternion_t;

/**
 * 3D (rotation) matrix.
 */
typedef union {
    float array[3][3];
    struct {
        float s11, s12, s13;
        float s21, s22, s23;
        float s31, s32, s33;
    };
} matrix3d_t;

#endif // COMMON_MATH_H