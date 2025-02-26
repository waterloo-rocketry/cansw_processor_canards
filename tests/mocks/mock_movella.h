#ifndef MOCK_MOVELLA_H
#define MOCK_MOVELLA_H

#include "fff.h"
#include "third_party/rocketlib/include/common.h"
#include "drivers/movella/movella.h"

// Declare FFF fakes for all movella functions
DECLARE_FAKE_VALUE_FUNC(w_status_t, movella_init);
DECLARE_FAKE_VALUE_FUNC(w_status_t, movella_get_data, movella_data_t *);

#endif // MOCK_MOVELLA_H 