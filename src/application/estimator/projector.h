#ifndef ESTIMATOR_PROJECTOR_H
#define ESTIMATOR_PROJECTOR_H

#include "application/controller/controller.h"
#include "application/estimator/estimator_types.h"

/**
 * @brief Computes roll state and scheduling variables for controller
 * @param est_state state vector containing attitude, rates, velocity, altitude, CL, and delta
 * @return vector with (1:3) state, (4:5) flight conditions
 */
controller_input_t projector(x_state_t *est_state);

#endif
