#include "log_mock.h"

// Define FFF mocks for logger functions
DEFINE_FAKE_VALUE_FUNC0(w_status_t, log_init);
DEFINE_FAKE_VALUE_FUNC_VARARG(w_status_t, log_text, uint32_t, const char *, const char *);
DEFINE_FAKE_VALUE_FUNC3(w_status_t, log_data, uint32_t, log_data_type_t, const log_data_container_t *);
DEFINE_FAKE_VOID_FUNC1(log_task, void *);