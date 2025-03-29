#ifndef GAIN_TABLE_H_
#define GAIN_TABLE_H_

#define GAIN_NUM 4

// for building each interpolation instance
#define GAIN_P_SIZE 200
#define GAIN_C_SIZE 30

// gain table variables
static const float pressure_dynamic_scale = 3.0619E+03;
static const float canard_coeff_scale = 6.6667E-01;

static const float pressure_dynamic_offset = 1.0000E+02;
static const float canard_coeff_offset = -1.0000E+01;

extern const float gain_table[GAIN_NUM][GAIN_P_SIZE * GAIN_C_SIZE];

#endif // GAIN_TABLE_H_