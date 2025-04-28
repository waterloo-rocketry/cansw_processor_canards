#ifndef JACOBIANS_H
#define JACOBIANS_H

// JACOBIANS HELPER FUNCTIONS

#include "application/estimator/estimator_types.h"

/**
 * @brief jacobians matrix definitions
 * @example rotation matrix 3x4: SIZE_VECTOR_3D * SIZE_QUAT
 * @example quaternion update matrix q_q 4x4: SIZE_QUAT * SIZE_QUAT
 * @example quaternion update matrix q_w 4x3: SIZE_QUAT * SIZE_VECTOR_3D
 * @example matrix3d_t 3x3: SIDE_MATRIX_3D * SIDE_MATRIX_3D
 * @example vector3d_t 3x1: SIZE_VECTOR_3D * SIZE_1D
 * @example 7x13 matrix: MEASUREMENT_MODEL_SIZE * X_STATE_SIZE_ITEMS
 */

// vector flat dimension
#define SIZE_1D 1
// size of measurement model
#define MEASUREMENT_MODEL_SIZE 7

// MATRIX DEFS
// rotation matrix
typedef union {
    double flat[SIZE_VECTOR_3D * SIZE_QUAT];
    struct {
        double s11, s12, s13, s14;
        double s21, s22, s23, s24;
        double s31, s32, s33, s34;
    };
} rotation_jacobian_t;

// quaternion update 4x4 matrix
typedef union {
    double flat[SIZE_QUAT * SIZE_QUAT];
    struct {
        double q11, q12, q13, q14;
        double q21, q22, q23, q24;
        double q31, q32, q33, q34;
        double q41, q42, q43, q44;
    };
} quaternion_update_matrix_q_t;

// quaternion update 4x3 matrix
typedef union {
    double flat[SIZE_QUAT * SIZE_VECTOR_3D];
    struct {
        double q11, q12, q13;
        double q21, q22, q23;
        double q31, q32, q33;
        double q41, q42, q43;
    };
} quaternion_update_matrix_w_t;

// 7x13 jacobian returned by model imu jacobian
typedef union {
    double flat[MEASUREMENT_MODEL_SIZE * X_STATE_SIZE_ITEMS];
    struct {
        double s11, s12, s13, s14, s15, s16, s17, s18, s19, s1x;
        double s21, s22, s23, s24, s25, s26, s27, s28, s29, s2x;
        double s31, s32, s33, s34, s35, s36, s37, s38, s39, s3x;
        double s41, s42, s43, s44, s45, s46, s47, s48, s49, s4x;
        double s51, s52, s53, s54, s55, s56, s57, s58, s59, s5x;
        double s61, s62, s63, s64, s65, s66, s67, s68, s69, s6x;
        double s71, s72, s73, s74, s75, s76, s77, s78, s79, s7x;
        double s81, s82, s83, s84, s85, s86, s87, s88, s89, s8x;
        double s91, s92, s93, s94, s95, s96, s97, s98, s99, s9x;
        double sA1, sA2, sA3, sA4, sA5, sA6, sA7, sA8, sA9, sAx;
    };
} measurement_model_jacobian_t;

/**
 * @brief helper function to construct pData for jacobian matrix instance
 * @param pData pointer of flattened jacobian matrix to write to
 * @param start_coor_x starting coordinate x in 13x13 jacobian (0-indexed)
 * @param start_coor_y starting coordinate y in 13x13 jacobian (0-indexed)
 * @param num_row number of rows of sub-structure
 * @param num_col number of columns of sub-structure
 * @param flat_data pointer to the data of sub-structure in row major order
 *
 * 2D coor (x, y) flattening to 1D coor (index): index = x * num_col + y
 *
 * CAUTIOUS only writes structures of pData with width of 13
 */
void write_pData(
    double *pData, int start_coor_x, int start_coor_y, int num_row, int num_col,
    const double *flat_data
);

#endif