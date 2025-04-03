/**
 * @file simulator.c
 * @brief Implementation of HIL Simulator Data Processor
 */

#include "application/hil/simulator.h"
#include "FreeRTOS.h"
#include "application/estimator/estimator.h"
#include "task.h"
#include <string.h>

// Define the expected payload size based on the document
#define SIM_EXPECTED_PAYLOAD_SIZE 140

// Define payload offsets based on the document
enum simulator_data_offsets {
    SIM_ENCODER_CANARD_ANGLE_OFFSET = 0,
    SIM_MOVELLA_ACCEL_X_OFFSET = 4,
    SIM_MOVELLA_ACCEL_Y_OFFSET = 8,
    SIM_MOVELLA_ACCEL_Z_OFFSET = 12,
    SIM_MOVELLA_GYRO_X_OFFSET = 16,
    SIM_MOVELLA_GYRO_Y_OFFSET = 20,
    SIM_MOVELLA_GYRO_Z_OFFSET = 24,
    SIM_MOVELLA_MAG_X_OFFSET = 28,
    SIM_MOVELLA_MAG_Y_OFFSET = 32,
    SIM_MOVELLA_MAG_Z_OFFSET = 36,
    SIM_MOVELLA_PRESSURE_OFFSET = 40,
    SIM_MOVELLA_QUAT_0_OFFSET = 44,
    SIM_MOVELLA_QUAT_1_OFFSET = 48,
    SIM_MOVELLA_QUAT_2_OFFSET = 52,
    SIM_MOVELLA_QUAT_3_OFFSET = 56,
    SIM_LSM_ACCEL_X_OFFSET = 60,
    SIM_LSM_ACCEL_Y_OFFSET = 64,
    SIM_LSM_ACCEL_Z_OFFSET = 68,
    SIM_LSM_GYRO_X_OFFSET = 72,
    SIM_LSM_GYRO_Y_OFFSET = 76,
    SIM_LSM_GYRO_Z_OFFSET = 80,
    SIM_LSM_MAG_X_OFFSET = 84,
    SIM_LSM_MAG_Y_OFFSET = 88,
    SIM_LSM_MAG_Z_OFFSET = 92,
    SIM_LSM_PRESSURE_OFFSET = 96,
    SIM_ALTIMU_ACCEL_X_OFFSET = 100,
    SIM_ALTIMU_ACCEL_Y_OFFSET = 104,
    SIM_ALTIMU_ACCEL_Z_OFFSET = 108,
    SIM_ALTIMU_GYRO_X_OFFSET = 112,
    SIM_ALTIMU_GYRO_Y_OFFSET = 116,
    SIM_ALTIMU_GYRO_Z_OFFSET = 120,
    SIM_ALTIMU_MAG_X_OFFSET = 124,
    SIM_ALTIMU_MAG_Y_OFFSET = 128,
    SIM_ALTIMU_MAG_Z_OFFSET = 132,
    SIM_ALTIMU_PRESSURE_OFFSET = 136,
};

// Static storage for the latest control output
static float latest_canard_angle_rad = 0.0f;
static bool new_control_output_available = false;

/**
 * @brief Initialize the HIL simulator module.
 */
w_status_t simulator_init(void) {
    latest_canard_angle_rad = 0.0f;
    new_control_output_available = false;
    return W_SUCCESS;
}

/**
 * @brief Process incoming simulated sensor data.
 */
w_status_t simulator_process_data(const uint8_t *payload, uint16_t payload_size) {
    if (payload == NULL || payload_size != SIM_EXPECTED_PAYLOAD_SIZE) {
        return W_INVALID_PARAM;
    }

    estimator_all_imus_input_t imu_data = {0};
    // estimator_canard_feedback_t canard_data = {0}; // Type/functionality doesn't exist

    // Use RTOS tick count as timestamp
    uint32_t now_ms = xTaskGetTickCount();

    // --- Populate Canard Feedback Data --- (Removed as type/function doesn't exist)
    // canard_data.angle_rad = *((float *)(payload + SIM_ENCODER_CANARD_ANGLE_OFFSET));
    // canard_data.timestamp_ms = now_ms;
    // Assuming they are not critical for HIL for now.
    // canard_data.angle_deg = 0.0f; // Placeholder
    // canard_data.control_request_id = 0; // Placeholder

    // --- Populate Movella IMU Data ---
    imu_data.movella.timestamp_imu = now_ms;
    imu_data.movella.accelerometer.x = *((float *)(payload + SIM_MOVELLA_ACCEL_X_OFFSET));
    imu_data.movella.accelerometer.y = *((float *)(payload + SIM_MOVELLA_ACCEL_Y_OFFSET));
    imu_data.movella.accelerometer.z = *((float *)(payload + SIM_MOVELLA_ACCEL_Z_OFFSET));
    imu_data.movella.gyroscope.x = *((float *)(payload + SIM_MOVELLA_GYRO_X_OFFSET));
    imu_data.movella.gyroscope.y = *((float *)(payload + SIM_MOVELLA_GYRO_Y_OFFSET));
    imu_data.movella.gyroscope.z = *((float *)(payload + SIM_MOVELLA_GYRO_Z_OFFSET));
    imu_data.movella.magnometer.x = *((float *)(payload + SIM_MOVELLA_MAG_X_OFFSET));
    imu_data.movella.magnometer.y = *((float *)(payload + SIM_MOVELLA_MAG_Y_OFFSET));
    imu_data.movella.magnometer.z = *((float *)(payload + SIM_MOVELLA_MAG_Z_OFFSET));
    imu_data.movella.barometer = *((float *)(payload + SIM_MOVELLA_PRESSURE_OFFSET));
    // Assuming quaternion data is not directly used by estimator, but available if needed.
    imu_data.movella.is_dead = false;

    // --- Populate Polulu (AltIMU) IMU Data ---
    imu_data.polulu.timestamp_imu = now_ms;
    imu_data.polulu.accelerometer.x = *((float *)(payload + SIM_ALTIMU_ACCEL_X_OFFSET));
    imu_data.polulu.accelerometer.y = *((float *)(payload + SIM_ALTIMU_ACCEL_Y_OFFSET));
    imu_data.polulu.accelerometer.z = *((float *)(payload + SIM_ALTIMU_ACCEL_Z_OFFSET));
    imu_data.polulu.gyroscope.x = *((float *)(payload + SIM_ALTIMU_GYRO_X_OFFSET));
    imu_data.polulu.gyroscope.y = *((float *)(payload + SIM_ALTIMU_GYRO_Y_OFFSET));
    imu_data.polulu.gyroscope.z = *((float *)(payload + SIM_ALTIMU_GYRO_Z_OFFSET));
    imu_data.polulu.magnometer.x = *((float *)(payload + SIM_ALTIMU_MAG_X_OFFSET));
    imu_data.polulu.magnometer.y = *((float *)(payload + SIM_ALTIMU_MAG_Y_OFFSET));
    imu_data.polulu.magnometer.z = *((float *)(payload + SIM_ALTIMU_MAG_Z_OFFSET));
    imu_data.polulu.barometer = *((float *)(payload + SIM_ALTIMU_PRESSURE_OFFSET));
    imu_data.polulu.is_dead = false;

    // --- Update Estimator ---
    w_status_t status = W_SUCCESS;
    status |= estimator_update_inputs_imu(&imu_data);
    // status |= estimator_update_inputs_canard(&canard_data); // Function doesn't exist

    return status;
}

/**
 * @brief Get the latest control output (canard angle) to send back.
 */
w_status_t simulator_get_control_output(float *canard_angle) {
    if (canard_angle == NULL) {
        return W_INVALID_PARAM;
    }

    // Provide the latest angle if available
    // It's okay if it hasn't been updated this tick, send the last known command.
    *canard_angle = latest_canard_angle_rad;
    new_control_output_available = false; // Mark as consumed
    return W_SUCCESS;
}

/**
 * @brief Set the latest control output (called by the controller task).
 */
void simulator_set_control_output(float canard_angle) {
    // Atomically update the latest angle (not strictly necessary with one writer)
    taskENTER_CRITICAL();
    latest_canard_angle_rad = canard_angle;
    new_control_output_available = true;
    taskEXIT_CRITICAL();
}