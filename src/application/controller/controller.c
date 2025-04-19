#include "application/controller/controller.h"
#include "application/can_handler/can_handler.h"
#include "application/controller/controller_algorithm.h"
#include "application/flight_phase/flight_phase.h"
#include "application/hil/hil.h"
#include "application/logger/log.h"
#include "drivers/timer/timer.h"
#include "drivers/uart/uart.h"
#include "queue.h"
#include <math.h>
#include <string.h>

// CAN related includes - Ensure base types are included first
#include "canlib/can.h" // Defines can_msg_t
#include "canlib/message/msg_actuator.h" // Defines build_actuator_analog_cmd_msg
#include "canlib/message_types.h" // Defines can_msg_prio_t, ACTUATOR_CANARD_ANGLE etc.

// Need can_handler interface for sending
#include "application/can_handler/can_handler.h"

static QueueHandle_t internal_state_queue;
static QueueHandle_t output_queue;

#define CONTROLLER_CYCLE_TIMEOUT_MS 5

static controller_t controller_state = {0};
static controller_output_t controller_output = {0};
static controller_gain_t controller_gain = {0};

/**
 * @brief Sends the canard angle command via CAN.
 *
 * Builds an actuator command message using canlib and sends it via the CAN handler.
 * Assumes the angle should be sent as uint16_t in milliradians.
 *
 * @param canard_angle Commanded canard angle in radians.
 * @return w_status_t W_SUCCESS on success, W_FAILURE on failure.
 */
static w_status_t controller_send_can(float canard_angle) {
    can_msg_t msg;
    w_status_t status = W_FAILURE;
    uint16_t timestamp = 0; // Placeholder timestamp

    // Convert radians to milliradians and cast to uint16_t
    uint16_t angle_mrad = (uint16_t)(canard_angle * 1000.0f);

    // Build the actuator command message
    if (build_actuator_analog_cmd_msg(
            PRIO_HIGH, timestamp, ACTUATOR_CANARD_ANGLE, angle_mrad, &msg
        )) {
        // Send the message via the CAN HANDLER
        status = can_handler_transmit(&msg);
        if (status != W_SUCCESS) {
            log_text(5, "controller", "CAN send failed via handler");
        }
    } else {
        log_text(5, "controller", "Failed to build CAN actuator command");
    }

    // --------------------------------------------------
    // -------- BEGIN HIL HARNESS CODE -----------
    // --------------------------------------------------

    // Use the debug UART channel defined in the uart driver
    uart_channel_t hil_uart_channel = UART_DEBUG_SERIAL;
    uint32_t timeout_ms = 10; // Add a timeout for the UART write

    // Packet structure: Header (4 bytes) + Payload (4 bytes) + Footer (1 byte)
    uint8_t packet_buffer[4 + sizeof(float) + 1];
    const uint16_t packet_size = sizeof(packet_buffer);

    // Set header
    packet_buffer[0] = 'o';
    packet_buffer[1] = 'r';
    packet_buffer[2] = 'z';
    packet_buffer[3] = '!';

    // Copy float payload (ensure correct byte order - assuming little-endian)
    memcpy(&packet_buffer[4], &canard_angle, sizeof(float));

    // Set footer (last byte of packet)
    packet_buffer[packet_size - 1] = HIL_UART_FOOTER_CHAR;

    // Send the packet via UART driver
    if (uart_write(hil_uart_channel, packet_buffer, packet_size, timeout_ms) == W_SUCCESS) {
        return W_SUCCESS;
    } else {
        // Log error if UART write fails
        log_text(5, "controller", "HIL UART write failed");
        return W_FAILURE;
    }

    // --------------------------------------------------
    // -------- END HIL HARNESS CODE -----------
    // --------------------------------------------------

    return status;
}

/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return W_SUCCESS if initialization successful
 */
w_status_t controller_init(void) {
    // Create internal state queue (length = 1)
    internal_state_queue =
        xQueueCreate(1, sizeof(controller_input_t)); // estimator function to controller task
    output_queue = xQueueCreate(1, sizeof(controller_output_t));

    // check queue creation
    if (NULL == internal_state_queue || NULL == output_queue) {
        log_text(10, "controller", "queue creation failed");
        return W_FAILURE;
    }

    // avoid controller/estimator deadlock
    xQueueOverwrite(output_queue, &commanded_angle_zero);

    // return w_status_t state
    log_text(10, "controller", "initialization successful");
    return W_SUCCESS;
}

/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return W_FAILURE if validation/queueing fails
 */
w_status_t controller_update_inputs(controller_input_t *new_state) {
    xQueueOverwrite(internal_state_queue, new_state); // overwrite internal queue

    return W_SUCCESS;
}

/**
 * Get most recent control output (cammanded angle and timestamp) - called by
 * state estimation module
 * @param output Pointer to store output -> type defined in controller
 * @return W_FAILURE if no output available
 */
w_status_t controller_get_latest_output(controller_output_t *output) {
    if (pdPASS == xQueuePeek(output_queue, output, 0)) {
        return W_SUCCESS;
    }

    return W_FAILURE;
}

/**
 * Controller task function for RTOS
 */
void controller_task(void *argument) {
    (void)argument;

    while (true) {
        // no phase change track
        flight_phase_state_t current_phase = flight_phase_get_state();
        // HIL MODIFICATION: flight phase doesnt exist in sim so make state_pad the valid flight
        // phase
        if (STATE_PAD != current_phase) { // if not in proper state
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            // wait for new state data (5ms timeout)
            controller_input_t new_state_msg;
            if (pdPASS ==
                xQueueReceive(
                    internal_state_queue, &new_state_msg, pdMS_TO_TICKS(CONTROLLER_CYCLE_TIMEOUT_MS)
                )) {
                controller_state.current_state = new_state_msg;

                // no roll program for test flight

            } else {
                controller_state.data_miss_counter++;

                // TODO if number of data misses exceed threshold, transition to safe mode
            }

            // controller calc: interpolate
            if (W_SUCCESS != interpolate_gain(
                                 controller_state.current_state.pressure_dynamic,
                                 controller_state.current_state.canard_coeff,
                                 &controller_gain
                             )) {
                controller_output.commanded_angle =
                    commanded_angle_zero; // command zero when out of bound
                log_text(10, "controller", "flight conditions out of bound");
            } else {
                if (W_SUCCESS != get_commanded_angle(
                                     controller_gain,
                                     controller_state.current_state.roll_state.roll_state_arr,
                                     &controller_output.commanded_angle
                                 )) {
                    controller_output.commanded_angle = commanded_angle_zero;
                    log_text(10, "controller", "failed to get commanded angle");
                }
            }

            // update timestamp for controller output
            float current_timestamp_ms;
            if (W_SUCCESS != timer_get_ms(&current_timestamp_ms)) {
                current_timestamp_ms = 0.0f;
                log_text(10, "controller", "failed to get timestamp for controller output");
            }
            controller_output.timestamp = (uint32_t)current_timestamp_ms;

            // update output queue (for other internal modules)
            xQueueOverwrite(output_queue, &controller_output);

            // send command visa CAN + log status/errors
            if (W_SUCCESS != controller_send_can(controller_output.commanded_angle)) {
                log_text(10, "controller", "commanded angle failed to send via CAN");
            }
        }
    }
}

