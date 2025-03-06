#ifndef LIS3MDL_h
#define LIS3MDL_h

enum LIS3MDL_reg {
    LIS3_WHO_AM_I = 0x0F,

    LIS3_CTRL_REG1 = 0x20,
    LIS3_CTRL_REG2 = 0x21,
    LIS3_CTRL_REG3 = 0x22,
    LIS3_CTRL_REG4 = 0x23,
    LIS3_CTRL_REG5 = 0x24,

    LIS3_STATUS_REG = 0x27,
    LIS3_OUT_X_L = 0x28,
    LIS3_OUT_X_H = 0x29,
    LIS3_OUT_Y_L = 0x2A,
    LIS3_OUT_Y_H = 0x2B,
    LIS3_OUT_Z_L = 0x2C,
    LIS3_OUT_Z_H = 0x2D,
    LIS3_TEMP_OUT_L = 0x2E,
    LIS3_TEMP_OUT_H = 0x2F,
    LIS3_INT_CFG = 0x30,
    LIS3_INT_SRC = 0x31,
    LIS3_INT_THS_L = 0x32,
    LIS3_INT_THS_H = 0x33,
};

#endif