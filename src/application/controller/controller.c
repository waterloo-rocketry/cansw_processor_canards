#include "application/controller/controller.h"

#include "application/flight_phase/flight_phase.h"
#include "application/hil/hil.h"
#include "application/logger/log.h"
#include "drivers/uart/uart.h"
#include "queue.h"
#include <string.h>
#include <math.h> 

#include "canlib/can.h"
#include "canlib/message/message_types.h"
#include "canlib/message/msg_actuator.h"

static QueueHandle_t internal_state_queue;
static QueueHandle_t output_queue;

#define CONTROLLER_CYCLE_TIMEOUT_MS 5

static controller_t controller_state = {0};
static controller_input_t controller_input __attribute__((unused)) = {0};
static controller_output_t controller_output = {0};
static controller_gain_t controller_gain __attribute__((unused)) = {0};

#if HIL_ENABLED
/**
 * @brief Sends the commanded canard angle back to the simulation host via UART.
 * 
 * This function formats the angle into the HIL packet structure (? + float + \n)
 * and sends it using the UART driver.
 * 
 * @param canard_angle The commanded angle in radians.
 * @return w_status_t W_SUCCESS on success, W_FAILURE on error.
 */
static w_status_t controller_send_hil_uart(float canard_angle) {
    // Use the debug UART channel defined in the uart driver
    uart_channel_t hil_uart_channel = UART_DEBUG_SERIAL; 

    // Packet structure: Header (1 byte) + Payload (4 bytes) + Footer (1 byte)
    uint8_t packet_buffer[1 + sizeof(float) + 1];
    const uint16_t packet_size = sizeof(packet_buffer);

    // Set header
    packet_buffer[0] = HIL_UART_HEADER_CHAR;

    // Copy float payload (ensure correct byte order - assuming little-endian)
    memcpy(&packet_buffer[1], &canard_angle, sizeof(float));

    // Set footer
    packet_buffer[1 + sizeof(float)] = HIL_UART_FOOTER_CHAR;

    // Send the packet via UART driver
    if (uart_write(hil_uart_channel, packet_buffer, packet_size) == W_SUCCESS) {
        return W_SUCCESS;
    } else {
        // Log error if UART write fails
        log_text("controller", "HIL UART write failed");
        return W_FAILURE;
    }
}
#endif

/**
 * @brief Sends the canard angle command via CAN.
 *
 * Builds an actuator command message using canlib and sends it via the CAN driver.
 * Assumes the angle should be sent as uint16_t in milliradians.
 *
 * @param canard_angle Commanded canard angle in radians.
 * @return w_status_t W_SUCCESS on success, W_FAILURE on failure.
 */
static w_status_t controller_send_can(float canard_angle) {
    can_msg_t msg;
    w_status_t status = W_FAILURE;

    // Convert radians to milliradians and cast to uint16_t
    uint16_t angle_mrad = (uint16_t)(canard_angle * 1000.0f);

    // Build the actuator command message
    if (build_actuator_analog_cmd(PRIO_HIGH, ACTUATOR_CANARD_ANGLE, angle_mrad, &msg)) {
        // Send the message via the CAN driver
        status = can_send_msg(&msg);
        if (status != W_SUCCESS) {
             log_text("controller", "CAN send failed");
        }
    } else {
        log_text("controller", "Failed to build CAN actuator command");
    }

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
        log_text("controller", "queue creation failed");
        return W_FAILURE;
    }

    // avoid controller/estimator deadlock
    xQueueOverwrite(output_queue, &controller_output);

    // TODO gain instance init

    // return w_status_t state
    log_text("controller", "initialization successful");
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
        if (current_phase != STATE_ACT_ALLOWED) { // if not in proper state
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            // wait for new state data (5ms timeout)
            controller_input_t new_state_msg;
            if (pdPASS ==
                xQueueReceive(
                    internal_state_queue, &new_state_msg, pdMS_TO_TICKS(CONTROLLER_CYCLE_TIMEOUT_MS)
                )) {
                controller_state.current_state = new_state_msg;
                // TODO validate data

                // no roll program for test flight

            } else {
                controller_state.data_miss_counter++;

                // TODO if number of data misses exceed threshold, notify health check module
            }

            // TODO controller calc: interpolate
            // For HIL, assume the calculation is done and the result is in controller_output

            // update output queue (for other internal modules)
            xQueueOverwrite(output_queue, &controller_output);

            // --- HIL Modification ---
#if HIL_ENABLED
            // Send the commanded angle back to the HIL simulator via UART
            controller_send_hil_uart(controller_output.commanded_angle);
#else

            // send command visa CAN + log status/errors (Keep original CAN send if needed)
            if (W_SUCCESS != controller_send_can(controller_output.commanded_angle)) {
                log_text("controller", "commanded angle failed to send via CAN");
            }
#endif // HIL_ENABLED


        }
    }
}

