#include "fatfs.h"

// Define the mock SD handle
SD_HandleTypeDef hsd1;

DEFINE_FAKE_VALUE_FUNC(FRESULT, f_open, FIL *, const TCHAR *, BYTE);
DEFINE_FAKE_VALUE_FUNC(FRESULT, f_close, FIL *);
DEFINE_FAKE_VALUE_FUNC(FRESULT, f_read, FIL *, void *, UINT, UINT *);
DEFINE_FAKE_VALUE_FUNC(FRESULT, f_write, FIL *, const void *, UINT, UINT *);
DEFINE_FAKE_VALUE_FUNC(FRESULT, f_lseek, FIL *, FSIZE_t);
DEFINE_FAKE_VALUE_FUNC(FRESULT, f_unlink, const TCHAR *);
DEFINE_FAKE_VALUE_FUNC(FRESULT, f_mount, FATFS *, const TCHAR *, BYTE);
DEFINE_FAKE_VALUE_FUNC(FSIZE_t, f_size, FIL *);

// Add HAL SD mock function definition
DEFINE_FAKE_VALUE_FUNC(HAL_SD_CardStateTypeDef, HAL_SD_GetCardState, SD_HandleTypeDef*);
