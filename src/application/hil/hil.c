/**
 * @file hil.c
 * @brief Implementation of Hardware-in-the-Loop interface
 */

#include "application/hil/hil.h"
#include "FreeRTOS.h"
#include "core_cm7.h"
#include "task.h"
#include <string.h>
#include "application/can_handler/can_handler.h" 
#include "rocketlib/include/common.h" 
#include "canlib/message/msg_sensor.h"
#include "canlib/can.h"
#include "canlib/message_types.h" 

// External declaration for the function in the FreeRTOS kernel
extern BaseType_t xTaskIncrementTick(void);

// --- Struct Definitions ---
// Ensure this matches the structure sent by the simulation host (HIL Test Plan)
// Use __attribute__((packed)) to prevent padding
typedef struct __attribute__((packed)) {
    // Offset 0: Encoder
    float encoder_angle;    // rad
    // Offset 4: Movella
    float movella_accel_x;  // g
    float movella_accel_y;  // g
    float movella_accel_z;  // g
    float movella_gyro_x;   // dps
    float movella_gyro_y;   // dps
    float movella_gyro_z;   // dps
    float movella_mag_x;    // uT
    float movella_mag_y;    // uT
    float movella_mag_z;    // uT
    float movella_pressure; // bar
    float movella_quat_0;   // -
    float movella_quat_1;   // -
    float movella_quat_2;   // -
    float movella_quat_3;   // -
    // Offset 60: LSM - descoped
    // float lsm_accel_x;      // g
    // float lsm_accel_y;      // g
    // float lsm_accel_z;      // g
    // float lsm_gyro_x;       // dps
    // float lsm_gyro_y;       // dps
    // float lsm_gyro_z;       // dps
    // float lsm_mag_x;        // uT
    // float lsm_mag_y;        // uT
    // float lsm_mag_z;        // uT
    // float lsm_pressure;     // bar  
    // Offset 100: AltIMU
    float altimu_accel_x;   // g
    float altimu_accel_y;   // g
    float altimu_accel_z;   // g
    float altimu_gyro_x;    // dps
    float altimu_gyro_y;    // dps
    float altimu_gyro_z;    // dps
    float altimu_mag_x;     // uT
    float altimu_mag_y;     // uT
    float altimu_mag_z;     // uT
    float altimu_pressure;  // bar 
} HilDataPacket;

// --- HIL UART protocol constants ---

#define HIL_UART_PAYLOAD_SIZE (sizeof(HilDataPacket)) // Use the updated struct size
#define HIL_UART_FRAME_SIZE (1 + HIL_UART_PAYLOAD_SIZE + 1) // Header + Payload + Footer


// Placeholder for Canard - replace with your actual Canard setup access
// extern CanardInstance g_canard;

/**
 * @brief Processes incoming simulator data from HIL UART payload.
 *
 * Parses the data and sends corresponding CAN messages.
 * Marked static as it's only called internally by hil_process_uart_data.
 * 
 * @param data Pointer to the raw payload data.
 * @param data_len Length of the payload data (must match HIL_UART_PAYLOAD_SIZE).
 * @return w_status_t W_SUCCESS if processing is successful, error code otherwise.
 */
static w_status_t simulator_process_data_internal(const uint8_t *data, uint16_t data_len) {
    if (data == NULL || data_len < sizeof(HilDataPacket)) {
        return W_FAILURE; // Not enough data for a full packet
    }

    const HilDataPacket* packet = (const HilDataPacket*)data;

    // Use uint16_t for timestamp 
    uint16_t timestamp = 0; 
    can_msg_t can_msg; // Structure to hold the CAN message built by canlib
    w_status_t status = W_SUCCESS; // Track overall status

    // --- Convert all incoming float data to uint16_t --- 
    // Encoder
    uint16_t encoder_angle = (uint16_t)packet->encoder_angle;
    // Movella
    uint16_t movella_accel_x = (uint16_t)packet->movella_accel_x;
    uint16_t movella_accel_y = (uint16_t)packet->movella_accel_y;
    uint16_t movella_accel_z = (uint16_t)packet->movella_accel_z;
    uint16_t movella_gyro_x = (uint16_t)packet->movella_gyro_x;
    uint16_t movella_gyro_y = (uint16_t)packet->movella_gyro_y;
    uint16_t movella_gyro_z = (uint16_t)packet->movella_gyro_z;
    uint16_t movella_mag_x = (uint16_t)packet->movella_mag_x;
    uint16_t movella_mag_y = (uint16_t)packet->movella_mag_y;
    uint16_t movella_mag_z = (uint16_t)packet->movella_mag_z;
    // LSM
    // uint16_t lsm_accel_x = (uint16_t)packet->lsm_accel_x;
    // uint16_t lsm_accel_y = (uint16_t)packet->lsm_accel_y;
    // uint16_t lsm_accel_z = (uint16_t)packet->lsm_accel_z;
    // uint16_t lsm_gyro_x = (uint16_t)packet->lsm_gyro_x;
    // uint16_t lsm_gyro_y = (uint16_t)packet->lsm_gyro_y;
    // uint16_t lsm_gyro_z = (uint16_t)packet->lsm_gyro_z;
    // uint16_t lsm_mag_x = (uint16_t)packet->lsm_mag_x;
    // uint16_t lsm_mag_y = (uint16_t)packet->lsm_mag_y;
    // uint16_t lsm_mag_z = (uint16_t)packet->lsm_mag_z;
    // AltIMU
    uint16_t altimu_accel_x = (uint16_t)packet->altimu_accel_x;
    uint16_t altimu_accel_y = (uint16_t)packet->altimu_accel_y;
    uint16_t altimu_accel_z = (uint16_t)packet->altimu_accel_z;
    uint16_t altimu_gyro_x = (uint16_t)packet->altimu_gyro_x;
    uint16_t altimu_gyro_y = (uint16_t)packet->altimu_gyro_y;
    uint16_t altimu_gyro_z = (uint16_t)packet->altimu_gyro_z;
    uint16_t altimu_mag_x = (uint16_t)packet->altimu_mag_x;
    uint16_t altimu_mag_y = (uint16_t)packet->altimu_mag_y;
    uint16_t altimu_mag_z = (uint16_t)packet->altimu_mag_z;
    // Pressure 
    uint16_t movella_pressure = (uint16_t)packet->movella_pressure;
    uint16_t altimu_pressure = (uint16_t)packet->altimu_pressure;
    // ... (quat conversions needed later)


    // --- Send Movella IMU Data (Accel/Gyro) ---
    // X-axis
    if (build_imu_data_msg(PRIO_HIGH, timestamp, 'x', IMU_PROC_MTI630, movella_accel_x, movella_gyro_x, &can_msg)) {
       if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }
    // Y-axis
    if (build_imu_data_msg(PRIO_HIGH, timestamp, 'y', IMU_PROC_MTI630, movella_accel_y, movella_gyro_y, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }
    // Z-axis
    if (build_imu_data_msg(PRIO_HIGH, timestamp, 'z', IMU_PROC_MTI630, movella_accel_z, movella_gyro_z, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }

    // --- Send Movella Magnetometer Data ---
    // X-axis
    if (build_mag_data_msg(PRIO_HIGH, timestamp, 'x', IMU_PROC_MTI630, movella_mag_x, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }
    // Y-axis
    if (build_mag_data_msg(PRIO_HIGH, timestamp, 'y', IMU_PROC_MTI630, movella_mag_y, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }
    // Z-axis
    if (build_mag_data_msg(PRIO_HIGH, timestamp, 'z', IMU_PROC_MTI630, movella_mag_z, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }

    // --- Send AltIMU IMU Data (Accel/Gyro) ---
    // X-axis
    if (build_imu_data_msg(PRIO_HIGH, timestamp, 'x', IMU_PROC_ALTIMU10, altimu_accel_x, altimu_gyro_x, &can_msg)) {
       if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }
    // Y-axis
    if (build_imu_data_msg(PRIO_HIGH, timestamp, 'y', IMU_PROC_ALTIMU10, altimu_accel_y, altimu_gyro_y, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }
    // Z-axis
    if (build_imu_data_msg(PRIO_HIGH, timestamp, 'z', IMU_PROC_ALTIMU10, altimu_accel_z, altimu_gyro_z, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }

    // --- Send AltIMU Magnetometer Data ---
    // X-axis
    if (build_mag_data_msg(PRIO_HIGH, timestamp, 'x', IMU_PROC_ALTIMU10, altimu_mag_x, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }
    // Y-axis
    if (build_mag_data_msg(PRIO_HIGH, timestamp, 'y', IMU_PROC_ALTIMU10, altimu_mag_y, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }
    // Z-axis
    if (build_mag_data_msg(PRIO_HIGH, timestamp, 'z', IMU_PROC_ALTIMU10, altimu_mag_z, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }

    // --- Send Pressure Data ---
    // Movella Pressure
    if (build_analog_data_msg(PRIO_MEDIUM, timestamp, SENSOR_BARO_PRESSURE, movella_pressure, &can_msg)) { // Using MEDIUM priority for pressure
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }
    // AltIMU Pressure
    if (build_analog_data_msg(PRIO_MEDIUM, timestamp, SENSOR_BARO_PRESSURE, altimu_pressure, &can_msg)) { 
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }

    // --- Send Movella Quaternion Data ---
    // Q0
    if (build_state_est_data_msg(PRIO_HIGH, timestamp, STATE_ID_ATT_Q0, &packet->movella_quat_0, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }
    // Q1
    if (build_state_est_data_msg(PRIO_HIGH, timestamp, STATE_ID_ATT_Q1, &packet->movella_quat_1, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }
    // Q2
    if (build_state_est_data_msg(PRIO_HIGH, timestamp, STATE_ID_ATT_Q2, &packet->movella_quat_2, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }
    // Q3
    if (build_state_est_data_msg(PRIO_HIGH, timestamp, STATE_ID_ATT_Q3, &packet->movella_quat_3, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }

    // --- Send Encoder Data ---
    if (build_analog_data_msg(PRIO_HIGH, timestamp, SENSOR_CANARD_ENCODER_1, encoder_angle, &can_msg)) {
        if (can_send_msg(&can_msg) != W_SUCCESS) { status = W_FAILURE; }
    } else { status = W_FAILURE; }

    return status; // Return overall status
}

/**
 * @brief Initializes the simulator module.
 * 
 */
w_status_t simulator_init() {
    // Initialize simulator specific things if needed
    // e.g., Initialize Canard instance if it's managed here/globally accessible
    return W_SUCCESS;
}

// --- HIL Mechanism Logic --- 

/**
 * @brief Manually advance the RTOS tick counter by one
 *
 * This function manually increments the FreeRTOS tick counter by one,
 * bypassing the normal SysTick interrupt-based tick mechanism.
 *
 * @return true if a context switch is requested, false otherwise
 */
bool hil_increment_tick(void) {
    BaseType_t xSwitchRequired = pdFALSE;

    // Disable interrupts before manipulating the tick counter
    portDISABLE_INTERRUPTS();

    // Increment the RTOS tick counter
    xSwitchRequired = xTaskIncrementTick();

    // If a context switch is required, pend the PendSV interrupt
    if (xSwitchRequired != pdFALSE) {
        // This is what would normally happen in the SysTick handler
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }

    // Re-enable interrupts
    portENABLE_INTERRUPTS();

    return (xSwitchRequired != pdFALSE);
}

/**
 * @brief Process HIL UART data and trigger tick updates
 *
 * This function should be called from the UART receive interrupt
 * handler. It validates the frame, processes simulator data,
 * and triggers the tick update.
 *
 * @param data Pointer to the received UART data (including header/footer)
 * @param size Size of the received data (including header/footer)
 * @return Status of the processing (W_SUCCESS, W_INVALID_PARAM)
 */
w_status_t hil_process_uart_data(const uint8_t *data, uint16_t size) {
    if (data == NULL) {
        return W_INVALID_PARAM;
    }
    
    // Validate frame size - should match the exact expected size
    if (size != HIL_UART_FRAME_SIZE) { 
        return W_INVALID_PARAM;
    }
    
    // Check header and footer
    if (data[0] != HIL_UART_HEADER_CHAR || data[size - 1] != HIL_UART_FOOTER_CHAR) {
        return W_INVALID_PARAM;
    }
    
    // Process the simulator data (payload is between header and footer)
    // Use HIL_UART_PAYLOAD_SIZE which should be based on HilDataPacket now
    w_status_t sim_status = simulator_process_data_internal(data + 1, HIL_UART_PAYLOAD_SIZE); 

    // Increment the RTOS tick counter regardless of simulator processing result
    hil_increment_tick();
    
    // Return success only if frame validation AND simulator processing were OK
    return (sim_status == W_SUCCESS) ? W_SUCCESS : W_FAILURE;
}