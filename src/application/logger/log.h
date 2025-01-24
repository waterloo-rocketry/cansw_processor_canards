#ifndef LOG_H
#define LOG_H

#include "rocketlib/include/common.h"

// TODO: Determine optimal numbers for these
/* Size of a single buffer (bytes) */
#define LOG_BUFFER_SIZE 16384
/* Size of each message region in text buffers (bytes) */
#define MAX_TEXT_MSG_LENGTH 256
/* Size of each message region in data buffers (bytes) */
#define MAX_DATA_MSG_LENGTH 64
/* Number of message regions in a single text buffer */
#define TEXT_MSGS_PER_BUFFER LOG_BUFFER_SIZE / MAX_TEXT_MSG_LENGTH
/* Number of message regions in a single data buffer */
#define DATA_MSGS_PER_BUFFER LOG_BUFFER_SIZE / MAX_DATA_MSG_LENGTH

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
 * All possible types of log messages created by log_data()
 */
typedef enum {
    LOG_TYPE_MOVELLA
    // TODO
} log_data_type_t;

/**
 * Create log buffers and mutexes necessary for logger operation
 */
w_status_t log_init(void);

/**
 * Log a message in text form to the text log file
 *
 * @param source A string identifying the source of the log message
 * @param format The message to log, optionally specifying printf-like formatting for optional
 * variable arguments
 * @param ... Optional values to print according to format
 * @return Status indicating success or failure
 */
w_status_t log_text(const char *source, const char *format, ...);

/**
 * Log a message in binary form to the data log file
 *
 * @param type The type of data log message to write
 * @param ... Data to write according to log.c's LOG_DATA_FORMATS[type] string
 * @return Status indicating success or failure
 */
w_status_t log_data(log_data_type_t type, ...);

void log_task(void *argument);

#endif
