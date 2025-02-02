#include "application/controller/controller.h"

QueueHandle_t internalStateQueue;
QueueHandle_t outputQueue;

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

// placer variables//////////////////////////////////////////////////////
typedef struct {
  /* data */
  float commanded_angle;
  uint32_t timestamp;
} estimator_controller_input_t;
//////////////////////////////////////////////////////////////////////

flight_condition_t flight_condition;
controller_t controller;
estimator_controller_input_t estimator_controller_input;

/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return W_SUCCESS if initialization successful
 *
    Initialize structs:
    Gain_table_t



    Log initialization status

 */
w_status_t controller_init(void) {
  bool init_status = true;

  // Create internal state queue (length = 1)
  internalStateQueue = xQueueCreate(1, sizeof(controller_input_t));

  // check queue creation
  if (internalStateQueue != NULL) {
    logInfo("controller", "queue creation successful");
  } else {
    logInfo("controller", "queue creation failed");
    init_status = false;
  }

  // Initialize structs
  // For `flight_condition`
  flight_condition.dynamic_pressure = 0;
  flight_condition.canard_coeff_lift = 0;

  // For `controller`
  controller.current_state.attitude.array[0] = 0;
  controller.current_state.attitude.array[1] = 0;
  controller.current_state.attitude.array[2] = 0;
  controller.current_state.attitude.array[3] = 0;

  controller.current_state.rates.component.x = 0;
  controller.current_state.rates.component.y = 0;
  controller.current_state.rates.component.z = 0;

  controller.current_state.velocity.component.x = 0;
  controller.current_state.velocity.component.y = 0;
  controller.current_state.velocity.component.z = 0;

  controller.current_state.altitude = 0;
  controller.current_state.timestamp = 0;
  controller.current_state.canard_coeff_CL = 0;
  controller.current_state.canard_angle_delta = 0;

  controller.controller_active = false;
  controller.last_ms = 0;
  controller.can_send_errors = 0;
  controller.data_miss_counter = 0;

  // For `estimator_controller_inputs`
  estimator_controller_input.commanded_angle = 0;
  estimator_controller_input.timestamp = 0;

  // check struct initialization

  // Log initialization status
  if (init_status) {
    logInfo("controller", "initialization successful");
  } else {
    logInfo("controller", "initialization failed");
    return W_FAILURE;
  }

  // return w_status_t state
  return W_SUCCESS;
}

/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return W_FAILURE if validation/queueing fails
 */
w_status_t controller_update_inputs(controller_input_t *new_state) {
  controller_input_t new_state_msg;

  if (xQueueReceive(internalStateQueue, &new_state_msg, 0) == pdPASS) {
    controller.current_state = new_state_msg;
    logInfo("controller", "controller inputs updated, current altitude %f",
            controller.current_state.altitude);
    return W_SUCCESS;
  }
  return W_FAILURE;
}

/**
 * Get most recent control output (cammanded angle and timestamp) - called by
 * state estimation module
 * @param output Pointer to store output -> type defined in state_estimation.h
 * @return W_FAILURE if no output available
 */
w_status_t controller_get_latest_output(estimator_controller_input_t *output) {
  xQueueCreate(outputQueue, sizeof(estimator_controller_input_t));
  if (xQueueSend(outputQueue, output, 0) == pdPASS) {
    logInfo("controller", "controller output sent, command angle %f",
            output->commanded_angle);
    return W_SUCCESS;
  }
  return W_FAILURE;
}

/**
 * Controller task function for RTOS
 */
void controller_task(void *argument) {}
