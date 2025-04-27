#ifndef ESTIMATOR_PROJECTOR_H
#define ESTIMATOR_PROJECTOR_H

#include "application/controller/controller.h"
#include "application/estimator/estimator_types.h"

/**
 * @brief Computes roll state and scheduling variables for controller
 * @param x_state_t state vector containing attitude, rates, velocity, altitude, CL, and delta
 * @return vector with (1:3) state, (4:5) flight conditions
 */
controller_input_t estimator_controller_projector(const x_state_t *state);

#endif
