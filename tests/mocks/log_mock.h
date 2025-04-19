#ifndef LOG_MOCK_H
#define LOG_MOCK_H

#include "fff.h"
#include "third_party/rocketlib/include/common.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

// Forward declarations
typedef enum {
    LOG_TYPE_NONE = 0,
    LOG_TYPE_ESTIMATOR_OUTPUT,
    LOG_TYPE_CONTROLLER_OUTPUT,
    LOG_TYPE_IMU_READING,
    // Other log types will be here in the real header
} log_data_type_t;

typedef union __attribute__((packed)) {
    // This would contain the actual data structures in the real header
    uint8_t dummy;
} log_data_container_t;

// Declarations of mock functions for logger
DECLARE_FAKE_VALUE_FUNC0(w_status_t, log_init);
DECLARE_FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *);
DECLARE_FAKE_VALUE_FUNC3(w_status_t, log_data, uint32_t, log_data_type_t, const log_data_container_t *);
DECLARE_FAKE_VOID_FUNC1(log_task, void *);

#endif // LOG_MOCK_H