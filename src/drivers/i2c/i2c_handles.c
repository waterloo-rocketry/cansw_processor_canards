#include "i2c_handles.h"

/* Provide dummy (weak) definitions of the I2C handles.
   If your firmware later provides proper definitions, you can remove or override these. */
I2C_HandleTypeDef hi2c1 = {0};
I2C_HandleTypeDef hi2c2 = {0};
I2C_HandleTypeDef hi2c3 = {0};
