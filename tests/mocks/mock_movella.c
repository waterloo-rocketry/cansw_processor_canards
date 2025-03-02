#include "mock_movella.h"

// Define the fake functions
DEFINE_FAKE_VALUE_FUNC(w_status_t, movella_init);
DEFINE_FAKE_VALUE_FUNC(w_status_t, movella_get_data, movella_data_t *);