#ifndef LOG_H
#define LOG_H

#include "rocketlib/include/common.h"
#include <stdint.h>

// TODO: Determine optimal numbers for these
/* Size of a single buffer (bytes) */
#define LOG_BUFFER_SIZE 16384
/* Size of each message region in text buffers (bytes) */
#define MAX_TEXT_MSG_LENGTH 256
/**
 * Size of each message region in data buffers (bytes).
 * If changing this value, make sure to update it in scripts/logparse.py too!
 */
#define MAX_DATA_MSG_LENGTH 64
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
    // Insert new types above this line in the format:
    // LOG_TYPE_XXX = M(unique_small_integer),
} log_data_type_t;

#undef M

/**
 * The container for data to be included in messages from log_data().
 * Make sure to update format descriptions in scripts/logparse.py too!
 */
typedef union {
    struct {
        uint32_t version;
        uint32_t index;
    } header;
    // Add structs for each type defined in log_data_type_t
} __attribute__((packed)) log_data_container_t;

/**
 * A collection of status variables describing the current health of the logger module.
 */
typedef struct {
    bool is_init;
    uint32_t dropped_msgs;
    uint32_t trunc_msgs;
    uint32_t full_buffer_moments;
    uint32_t log_write_timeouts;
    uint32_t invalid_region_moments;
    uint32_t crit_errs;
    uint32_t no_full_buf_moments;
    uint32_t buffer_flush_fails;
} logger_health_t;

/**
 * @brief Create log buffers and mutexes necessary for logger operation.
 */
w_status_t log_init(void);

/**
 * @brief Log a message in text form to the text log file.
 *
 * @param timeout Maximum amount of time to block for, in ms
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
 * @param timeout Maximum amount of time to block for, in ms
 * @param type The type of data log message to write
 * @param data Pointer to raw data to write via memcpy
 * @return Status indicating success or failure
 */
w_status_t log_data(uint32_t timeout, log_data_type_t type, const log_data_container_t *data);

void log_task(void *argument);

#endif
