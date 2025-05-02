#include "application/estimator/estimator_module.h"
#include "application/logger/log.h"

w_status_t estimator_module(
    const estimator_module_input_t *input, flight_phase_state_t flight_phase,
    estimator_module_ctx_t *ctx, controller_input_t *output_to_controller
) {
    w_status_t status = W_SUCCESS;

    switch (flight_phase) {
        // ------- if idle state: do nothing -------
        case STATE_IDLE:
            // do nothing.
            break;

        // ------- after SE-init CAN msg received: run pad filter -------
        case STATE_SE_INIT:
            // initialize the pad filter if it hasn't been done yet
            if (false == ctx->pad_filter_ctx.is_initialized) {
                status |= pad_filter_init(
                    &ctx->pad_filter_ctx,
                    &input->movella,
                    &input->pololu,
                    input->movella_is_dead,
                    input->pololu_is_dead
                );

                if (status == W_SUCCESS) {
                    log_text(5, "Estimator", "Pad filter init!");
                } else {
                    log_text(10, "Estimator", "Pad filter init fail");
                }
            }

            // run pad filter
            status |= pad_filter(
                &ctx->pad_filter_ctx,
                &input->movella,
                &input->pololu,
                input->movella_is_dead,
                input->pololu_is_dead,
                &ctx->x,
                &ctx->bias_movella,
                &ctx->bias_pololu
            );

            if (status != W_SUCCESS) {
                log_text(10, "Estimator", "ERROR: Pad filter run failed.");
            }
            break;

        // ------- flight!! perform a state estimation cycle -------
        case STATE_BOOST:
        case STATE_ACT_ALLOWED:
        case STATE_RECOVERY:

            // TODO: run ekf

            // ekf_algorithm(
            //     &g_x,
            //     &g_P,
            //     &latest_controller_cmd,
            //     &latest_imu_data.movella,
            //     &g_pad_filter_ctx.filtered_1,
            //     &latest_imu_data.pololu,
            //     &g_pad_filter_ctx.filtered_2,
            //     &latest_encoder_data
            // );

            // TODO: Remove this dummy state once EKF is implemented
            // Populate with some dummy values for testing
            ctx->x.attitude.w = 1.0f;
            ctx->x.rates.x = 0.1f;
            ctx->x.velocity.z = -9.8f;
            ctx->x.altitude = 100.0f;
            ctx->x.CL = 0.5f;
            ctx->x.delta = 0.05f;
            // END DUMMY STATE
            break;

        default:
            log_text(10, "Estimator", "invalid flight phase: %d", flight_phase);
            return W_FAILURE;
            break;
    }

    return status;
}
