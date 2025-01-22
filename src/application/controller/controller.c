#include "controller.h"


QueueHandle_t internalStateQueue;
/**
 * Initialize controller module
 * Must be called before RTOS scheduler starts
 * @return true if initialization successful
 */
bool controller_init(void) {

    // 1. Create internal state queue
    internalStateQueue = xQueueCreate(2, sizeof(float));
    if (internalStateQueue == NULL) {
        logInfo("controller", "Error: Internal state queue creation failed.\n");
        return false;
    }

    // 2. Initialize structs

    // 2.1 Gain table and flight conditions would go here (not shown)

    // 2.2 Controller initialization with memory checks
    float* g0 = (float*)malloc(sizeof(float));
    if (g0 == NULL) { 
        logInfo("controller", "Error: Memory allocation for g0 failed.\n"); 
        return false; 
    }
    *g0 = 9.8f;  // Zero height gravity

    float* air_R = (float*)malloc(sizeof(float));
    if (air_R == NULL) { 
        logInfo("controller", "Error: Memory allocation for air_R failed.\n"); 
         
        return false; 
    }
    *air_R = 287.0579f;  // Specific gas constant for air

    float* T_B = (float*)malloc(sizeof(float));
    if (T_B == NULL) { 
        logInfo("controller", "Error: Memory allocation for T_B failed.\n"); 
         
        return false; 
    }
    *T_B = 288.15f;  // Troposphere base temperature

    float* P_B = (float*)malloc(sizeof(float));
    if (P_B == NULL) { 
        logInfo("controller", "Error: Memory allocation for P_B failed.\n"); 
        
        return false; 
    }
    *P_B = 101325.0f;  // Troposphere base pressure

    float* P = (float*)malloc(sizeof(float));
    if (P == NULL) { 
        logInfo("controller", "Error: Memory allocation for P failed.\n"); 
         
        return false; 
    }
    *P = 101300.0f;  // Initial pressure      

    // Compute quaternion attitude
    float* phi = (float*)malloc(sizeof(float));  // Initial pitch angle -- deg2rad(-5)
    if (phi == NULL) { 
        logInfo("controller", "Error: Memory allocation for phi failed.\n");
         
        return false;
    }
    *phi = (-5.0f) * M_PI / 180.0f;  // Convert -5 degrees to radians

    float* d = (float*)malloc(3 * sizeof(float));  // Axis of rotation
    if (d == NULL) {
        logInfo("controller", "Error: Memory allocation for d failed.\n");
        
        return false;
    }
    d[0] = 0.0f;
    d[1] = 1.0f;
    d[2] = 0.0f;

    // Initialize controller_t structure
    controller_t controller_var = { 
        .current_state = {
            .attitude = { 
                .w = cos(*phi / 2),
                .x = d[0] * sin(*phi / 2),
                .y = d[1] * sin(*phi / 2),
                .z = d[2] * sin(*phi / 2)
            },
            .rates = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
            .velocity = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
            .altitude = -log((*P) / (*P_B)) * (*air_R) * (*T_B) / (*g0),
            .timestamp = 0 
        },
        .last_error = FLIGHT_PHASE_NONE, 
        .controller_active = false,
        .last_ms = 0,
        .can_send_errors = 0,
        .data_miss_counter = 0
    };

    // Free up memory after controller initialization
    free(g0);
    free(air_R);
    free(T_B);
    free(P_B);
    free(P);
    free(phi);
    free(d);

    // 2.4 controller_output_SE_t 
    control_output_SE_t state_est_output = {
        .command_angle = 0.0f,
        .timestamp = 0
    };

    // 3. Log initialization status
    logInfo("controller", "Controller initialized.\n");
    return true;
}



/**
 * Update controller with new state data - called by state estimation module
 * @param new_state Latest state estimate from state estimation
 * @return false if validation/queueing fails
 */
bool controller_update_inputs(controller_state_t *new_state){

}

/**
 * Get most recent control output - called by state estimation module
 * @param output Pointer to store output -> type defined in state_estimation.h
 * @return false if no output available
 */
bool controller_get_latest_output(control_output_SE_t *output){

}

/**
 * Controller task function for RTOS
 */
void controller_task(void *argument){
    
}





