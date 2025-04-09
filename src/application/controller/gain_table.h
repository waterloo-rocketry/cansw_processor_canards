#ifndef GAIN_TABLE_H_
#define GAIN_TABLE_H_

#define GAIN_NUM 4

// for building each interpolation instance
#define GAIN_P_SIZE 200
#define GAIN_C_SIZE 30

// gain table variables

#define PRESSURE_DYNAMIC_SCALE 2.7512E+02
#define CANARD_COEFF_SCALE 5.0000E-01

#define PRESSURE_DYNAMIC_OFFSET 1.0000E+02
#define CANARD_COEFF_OFFSET -5.0000E+00

extern const float gain_table[GAIN_NUM][GAIN_P_SIZE * GAIN_C_SIZE];

#endif // GAIN_TABLE_H_