/**
 * @file simulator.h
 * @brief Hardware-in-the-Loop (HIL) Simulator Data Processor
 * 
 * Parses incoming sensor data from the HIL simulation (MATLAB)
 * and provides control outputs back to the simulation.
 */

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "rocketlib/include/common.h"
#include "application/estimator/estimator.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the HIL simulator module.
 * 
 * @return w_status_t Status code.
 */
w_status_t simulator_init(void);

/**
 * @brief Process incoming simulated sensor data.
 * 
 * Parses the data payload received via UART from the HIL simulation,
 * populates the estimator input structure, and updates the estimator.
 * 
 * @param payload Pointer to the start of the data payload (after header).
 * @param payload_size Size of the data payload (excluding header/footer).
 * @return w_status_t Status code (W_SUCCESS or W_INVALID_PARAM).
 */
w_status_t simulator_process_data(const uint8_t *payload, uint16_t payload_size);

/**
 * @brief Get the latest control output (canard angle) to send back.
 * 
 * @param canard_angle Pointer to a float where the angle (in radians) will be stored.
 * @return w_status_t Status code (W_SUCCESS if an angle is available).
 */
w_status_t simulator_get_control_output(float *canard_angle);

/**
 * @brief Set the latest control output (called by the controller task).
 * 
 * @param canard_angle The latest canard angle command (in radians).
 */
void simulator_set_control_output(float canard_angle);


#endif /* SIMULATOR_H */ 