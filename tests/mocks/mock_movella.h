#ifndef MOCK_MOVELLA_H
#define MOCK_MOVELLA_H

#include "drivers/movella/movella.h"
#include "fff.h"

FAKE_VALUE_FUNC(w_status_t, movella_init);
FAKE_VALUE_FUNC(w_status_t, movella_get_data, movella_data_t *);

#endif // MOCK_MOVELLA_H