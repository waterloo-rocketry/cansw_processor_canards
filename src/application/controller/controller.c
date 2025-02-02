#include "controller.h"

xQueueHandle_t internalStateQueue;

/*
Send `canard_angle`, the desired canard angle (radians) to CAN
*/
w_status_t controller_send_san(float canard_angle) {
  // Build the CAN msg using [canard-specific canlib function to be defined
  // later]. Send this to can handler module’s tx
}

// internal: flight_condition_t
typedef struct {
  float dynamic_pressure;
  float canard_coeff_lift;
} flight_condition_t;

//////////////////////////////////////////////////////////////////////

/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return W_SUCCESS if initialization successful
 */
w_status_t controller_init(void) {
  /*

    Initialize structs:
    Gain_table_t


    
    Log initialization status

  */

  // Create internal state queue (length = 1)
  internalStateQueue = xQueueCreate(1, sizeof(controller_input_t));
  if (internalStateQueue == NULL) {
    return W_FAILURE;
  }

  // Initialize structs
  flight_condition_t flight_condition = {.dynamic_pressure = 0,
                                         .canard_coeff_lift = 0};
  controller_t controller = {.current_state = {.attitude = {0, 0, 0, 0},
                                               .rates = {0, 0, 0},
                                               .velocity = {0, 0, 0},
                                               .altitude = 0,
                                               .timestamp = 0,
                                               .canard_coeff_CL = 0,
                                               .canard_angle_delta = 0},
                             .controller_active = false,
                             .last_ms = 0,
                             .can_send_errors = 0,
                             .data_miss_counter = 0};

  estimator_controller_inputs_t estimator_controller_inputs = {
    .command_angle = 0;
  .timestamp = 0;
};


}

/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return W_FAILURE if validation/queueing fails
 */
w_status_t controller_update_inputs(controller_input_t *new_state);

/**
 * Get most recent control output - called by state estimation module
 * @param output Pointer to store output -> type defined in state_estimation.h
 * @return W_FAILURE if no output available
 */
w_status_t controller_get_latest_output(estimator_controller_inputs_t *output);

/**
 * Controller task function for RTOS
 */
void controller_task(void *argument);
