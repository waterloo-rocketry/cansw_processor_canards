#ifndef LOG_H
#define LOG_H

#include "rocketlib/include/common.h"
#include <stdint.h>
// Include headers for structs used in log_data_container_t
#include "application/controller/controller.h" // For controller_input_t, controller_output_t
#include "application/estimator/estimator.h" // For estimator_all_imus_input_t
#include "application/imu_handler/imu_handler.h"

/* Size of a single buffer (bytes) */
#define LOG_BUFFER_SIZE 16384
/* Size of each message region in text buffers (bytes) */
#define MAX_TEXT_MSG_LENGTH 256
/**
 * Size of each message region in data buffers (bytes).
 * If changing this value, make sure to update it in scripts/logparse.py too!
 */
#define MAX_DATA_MSG_LENGTH 32
/* Number of message regions in a single text buffer */
#define TEXT_MSGS_PER_BUFFER (LOG_BUFFER_SIZE / MAX_TEXT_MSG_LENGTH)
/* Number of message regions in a single data buffer */
#define DATA_MSGS_PER_BUFFER (LOG_BUFFER_SIZE / MAX_DATA_MSG_LENGTH)

#if LOG_BUFFER_SIZE % MAX_TEXT_MSG_LENGTH != 0
#warning "Text log message region size does not pack evenly into buffer size"
#endif
#if LOG_BUFFER_SIZE % MAX_DATA_MSG_LENGTH != 0
#warning "Data log message region size does not pack evenly into buffer size"
#endif

/* Number of text log buffers */
#define NUM_TEXT_LOG_BUFFERS 2
/* Number of data log buffers */
#define NUM_DATA_LOG_BUFFERS 2

/**
 * Version number to identify post-flight how to parse data log messages.
 * Increment this value when making an incompatible change to log_data_type_t or log_data()'s
 * formatting of messages.
 *
 * Deprecated values: none
 */
#define LOG_DATA_FORMAT_VERSION 1

/**
 * Magic number to encode into log_data_type_t values: "DL" encoded as a little-endian 16-bit int.
 * If changing this value, make sure to update it in scripts/logparse.py too!
 */
#define LOG_DATA_MAGIC 0x4c44

/**
 * Place v in upper 16 bits and LOG_DATA_MAGIC in lower 16 bits.
 * If changing this macro, make sure to update it in scripts/logparse.py too!
 */
#define M(v) ((((v) & 0xffff) << 16) | LOG_DATA_MAGIC)

/**
 * All possible types of log messages emitted by log_data().
 * Make sure to update scripts/logparse.py too!
 *
 * Deprecated values: none
 */
typedef enum {
    LOG_TYPE_HEADER = 0x44414548, // "HEAD" encoded as a little-endian 32-bit int
    LOG_TYPE_TEST = M(0x01),
    LOG_TYPE_CANARD_CMD = M(0x02),

    LOG_TYPE_CONTROLLER_INPUT_PT1 = M(0x03),
    LOG_TYPE_CONTROLLER_INPUT_PT2 = M(0x04),
    LOG_TYPE_CONTROLLER_INPUT_PT3 = M(0x05),
    LOG_TYPE_CONTROLLER_INPUT_PT4 = M(0x06),
    LOG_TYPE_CONTROLLER_INPUT_PT5 = M(0x07),

    LOG_TYPE_MOVELLA_READING_PT1 = M(0x08),
    LOG_TYPE_MOVELLA_READING_PT2 = M(0x08),
    LOG_TYPE_MOVELLA_READING_PT3 = M(0x08),
    LOG_TYPE_MOVELLA_READING_PT4 = M(0x08),
    LOG_TYPE_MOVELLA_READING_PT5 = M(0x08),
    LOG_TYPE_MOVELLA_READING_PT6 = M(0x08),
    LOG_TYPE_MOVELLA_READING_PT7 = M(0x08),
    LOG_TYPE_MOVELLA_READING_PT8 = M(0x08),
    LOG_TYPE_MOVELLA_READING_PT9 = M(0x08),
    LOG_TYPE_MOVELLA_READING_PT10 = M(0x08),
    LOG_TYPE_MOVELLA_READING_PT11 = M(0x08),
    LOG_TYPE_MOVELLA_READING_PT12 = M(0x08),

    LOG_TYPE_ESTIMATOR_CTX_PT1 = M(0x09),
    LOG_TYPE_ESTIMATOR_CTX_PT2 = M(0x0A),
    LOG_TYPE_ESTIMATOR_CTX_PT3 = M(0x0B),
    LOG_TYPE_ESTIMATOR_CTX_PT4 = M(0x0C),
    LOG_TYPE_ESTIMATOR_CTX_PT5 = M(0x0D),
    LOG_TYPE_ESTIMATOR_CTX_PT6 = M(0x0E),
    LOG_TYPE_ESTIMATOR_CTX_PT7 = M(0x0F),
    LOG_TYPE_ESTIMATOR_CTX_PT8 = M(0x10),
    LOG_TYPE_ESTIMATOR_CTX_PT9 = M(0x11),
    LOG_TYPE_ESTIMATOR_CTX_PT10 = M(0x12),
    LOG_TYPE_ESTIMATOR_CTX_PT11 = M(0x13),
    LOG_TYPE_ESTIMATOR_CTX_PT12 = M(0x14),
    LOG_TYPE_ESTIMATOR_CTX_PT13 = M(0x15),
    LOG_TYPE_ESTIMATOR_CTX_PT14 = M(0x16),

    LOG_TYPE_ENCODER = M(0x17),

    LOG_TYPE_POLOLU_READING_PT1 = M(0x18),
    LOG_TYPE_POLOLU_READING_PT2 = M(0x19),
    LOG_TYPE_POLOLU_READING_PT3 = M(0x1A),
    LOG_TYPE_POLOLU_READING_PT4 = M(0x1B),
    LOG_TYPE_POLOLU_READING_PT5 = M(0x1C),
    LOG_TYPE_POLOLU_READING_PT6 = M(0x1D),
    LOG_TYPE_POLOLU_READING_PT7 = M(0x1E),
    LOG_TYPE_POLOLU_READING_PT8 = M(0x1F),
    LOG_TYPE_POLOLU_READING_PT9 = M(0x20),
    LOG_TYPE_POLOLU_READING_PT10 = M(0x21),
    LOG_TYPE_POLOLU_READING_PT11 = M(0x22),
    LOG_TYPE_POLOLU_READING_PT12 = M(0x23),

    LOG_TYPE_POLOLU_RAW = M(0x24),

    // Insert new types above this line in the format:
    // LOG_TYPE_XXX = M(unique_small_integer),
} log_data_type_t;

#undef M

/**
 * The container for data to be included in messages from log_data().
 * Make sure to update format descriptions in scripts/logparse.py too!
 */
// Add structs for each type defined in log_data_type_t
// Please include `__attribute__((packed))` in struct declarations
typedef union __attribute__((packed)) {
    // LOG_TYPE_HEADER:
    struct __attribute__((packed)) {
        uint32_t version;
        uint32_t index;
    } header;

    // LOG_TYPE_TEST:
    struct __attribute__((packed)) {
        float test_val;
    } test;

    // LOG_TYPE_CANARD_CMD:
    struct __attribute__((packed)) {
        float cmd_angle;
    } controller;

    // LOG_TYPE_CONTROLLER_INPUT:
    controller_input_t __attribute__((packed)) controller_input;

    // LOG_TYPE_MOVELLA_READING or LOG_TYPE_POLOLU_READING:
    // note: dont use the all_imus_input_t struct here because packing isn't recursive
    struct __attribute__((packed)) {
        uint32_t timestamp_imu;
        vector3d_f32_t accelerometer; // m/s^2
        vector3d_f32_t gyroscope; // rad/sec
        vector3d_f32_t magnetometer; // mgauss (pololu) or arbitrary units (movella)
        float barometer; // Pa
        bool is_dead;
    } imu_reading;

    // LOG_TYPE_ESTIMATOR_CTX:
    struct __attribute__((packed)) {
        x_state_f32_t x_array;

    } estimator_ctx_pt1_13;

    struct __attribute__((packed)) {
        float t;

    } estimator_ctx_pt14;

    // LOG_TYPE_ENCODER:
    float encoder;

    // LOG_TYPE_POLOLU_RAW:
    raw_pololu_data_t raw_pololu_data; // #################################CHANGE
} log_data_container_t;

/**
 * A collection of status variables describing the current health of the logger module.
 */
typedef struct {
    bool is_init;
    uint32_t dropped_txt_msgs;
    uint32_t dropped_data_msgs;
    uint32_t trunc_msgs;
    uint32_t full_buffer_moments;
    uint32_t log_write_timeouts;
    uint32_t invalid_region_moments;
    uint32_t crit_errs;
    uint32_t no_full_buf_moments;
    uint32_t buffer_flush_fails;
    uint32_t unsafe_buffer_flushes;
} logger_health_t;

/**
 * @brief Create log buffers and mutexes necessary for logger operation.
 * @pre must be run after scheduler start, and after sd_card module is init
 */
w_status_t log_init(void);

/**
 * @brief Log a message in text form to the text log file.
 *
 * @param timeout Maximum amount of time to block waiting to log, in ms
 * @param source A string identifying the source of the log message
 * @param format The message to log, optionally specifying printf-like formatting for optional
 * variable arguments
 * @param ... Optional values to print according to format
 * @return Status indicating success or failure
 */
w_status_t log_text(uint32_t timeout, const char *source, const char *format, ...);

/**
 * @brief Log a message in binary form to the data log file.
 *
 * @param timeout Maximum amount of time to block waiting to log, in ms
 * @param type The type of data log message to write
 * @param data Pointer to raw data to write via memcpy
 * @return Status indicating success or failure
 */
w_status_t log_data(uint32_t timeout, log_data_type_t type, const log_data_container_t *data);

void log_task(void *argument);

#endif
