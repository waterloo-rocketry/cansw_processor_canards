#include <stdbool.h>

#include "queue.h"
#include "semphr.h"
#include "rocketlib/include/common.h"
#include "application/logger/log.h"
#include "third_party/printf/printf.h"

#define LOG_RUN_COUNT_FILENAME "logrun"

typedef struct {
	bool is_text;
	bool is_full;
	uint32_t current_msg_num;
	SemaphoreHandle_t current_msg_num_mutex;
	SemaphoreHandle_t msgs_done_semaphore;
	char data[LOG_BUFFER_SIZE];
} log_buffer_t;

static log_buffer_t log_text_buffers[NUM_TEXT_LOG_BUFFERS];
static log_buffer_t log_data_buffers[NUM_DATA_LOG_BUFFERS];
static uint32_t current_text_buf_num = 0;
static uint32_t current_data_buf_num = 0;
static SemaphoreHandle_t log_text_write_mutex = NULL;
static SemaphoreHandle_t log_data_write_mutex = NULL;
static QueueHandle_t full_buffers_queue = NULL;

static struct {
	uint32_t dropped_msgs;
	uint32_t trunc_msgs;
	uint32_t trunc_msgs;
	uint32_t dropped_msgs;
	uint32_t full_buffer_moments;
	uint32_t log_write_timeouts;
	uint32_t invalid_region_moments;
	uint32_t crit_errs;
	uint32_t no_full_buf_moments;
} log_status = {0};

static void log_reset_buffer(log_buffer_t *buffer) {
	buffer->is_full = false;
	buffer->current_msg_num = 0;
	buffer->current_msg_num_mutex = xSemaphoreCreateMutex();
	buffer->msgs_done = 0;
	memset(buffer->data, 0, LOG_BUFFER_SIZE);
}

w_status_t log_init(void) {
	full_buffers_queue = xQueueCreate(NUM_TEXT_LOG_BUFFERS + NUM_DATA_LOG_BUFFERS, sizeof(log_buffer_t *));
	if (NULL == full_buffers_queue) {
		return W_FAILURE;
	}

	log_text_write_mutex = xSemaphoreCreateMutex();
	if (NULL == log_text_write_mutex) {
		return W_FAILURE;
	}

	log_data_write_mutex = xSemaphoreCreateMutex();
	if (NULL == log_data_write_mutex) {
		return W_FAILURE;
	}

	for (int i = 0; i < NUM_TEXT_LOG_BUFFERS; i++) {
		log_buffer_t buffer = log_text_buffers[i];
		buffer.is_text = true;
		buffer.current_msg_num_mutex = xSemaphoreCreateMutex();
		if (NULL == buffer.current_msg_num_mutex) {
			return W_FAILURE;
		}
		buffer.msgs_done_semaphore = xSemaphoreCreateCounting(TEXT_MSGS_PER_BUFFER, 0);
		if (NULL == buffer.msgs_done_semaphore) {
			return W_FAILURE;
		}
		log_reset_buffer(&buffer);
	}

	for (int i = 0; i < NUM_DATA_LOG_BUFFERS; i++) {
		log_buffer_t buffer = log_data_buffers[i];
		buffer.is_text = false;
		buffer.current_msg_num_mutex = xSemaphoreCreateMutex();
		if (NULL == buffer.current_msg_num_mutex) {
			return W_FAILURE;
		}
		buffer.msgs_done_semaphore = xSemaphoreCreateCounting(DATA_MSGS_PER_BUFFER, 0);
		if (NULL == buffer.msgs_done_semaphore) {
			return W_FAILURE;
		}
		log_reset_buffer(&buffer);
	}

	return W_SUCCESS;
}

w_status_t log_text(const char *source, const char *format, ...) {
	float timestamp = -1.0f;
	// If we fail to get a timestamp, use a dummy value of -1.0f and continue to write the log message anyway
	// We're trying to log as much as possible and a missing timestamp does not inherently critically affect the ability to write the log message
	timer_get_ms(&timestamp);

	if (xSemaphoreTake(log_text_write_mutex, 10) != pdPASS) {
		log_status.log_write_timeouts++;
		log_status.dropped_msgs++;
		return W_FAILURE;
	}

	log_buffer_t *buffer = &log_text_buffers[current_text_buf_num];

	if (buffer->is_full) {
		xSemaphoreGive(log_text_write_mutex);
		log_status.full_buffer_moments++;
		log_status.dropped_msgs++;
		return W_FAILURE;
	}

	const uint32_t msg_num = buffer->current_msg_num++;

	if (msg_num == TEXT_MSGS_PER_BUFFER - 1) {
		buffer->is_full = true;
		current_text_buf_num = (current_text_buf_num + 1) % NUM_TEXT_LOG_BUFFERS;
	}

	xSemaphoreGive(log_text_write_mutex);

	if (msg_num >= TEXT_MSGS_PER_BUFFER) {
		log_status.invalid_region_moments++;
		log_status.dropped_msgs++;
		return W_FAILURE;
	}

	char *const msg_dest = buffer->data + msg_num * MAX_TEXT_MSG_LENGTH;

	uint32_t chars_written = 0;
	chars_written += snprintf_(msg_dest + chars_written, MAX_TEXT_MSG_LENGTH - chars_written, "[%f] %s;", timestamp, source);
	if (chars_written >= MAX_TEXT_MSG_LENGTH) {
		msg_dest[0] = '!';
		log_status.trunc_msgs++;
	} else {
		va_list format_args;
		va_start(format_args, format);
		chars_written += vsnprintf_(msg_dest + chars_written, MAX_TEXT_MSG_LENGTH - chars_written, format, format_args);
		va_end(format_args);
		if (chars_written >= MAX_TEXT_MSG_LENGTH) {
			msg_dest[0] = '!';
			log_status.trunc_msgs++;
		}
	}
	// Remaining bytes are zeroed by task after flushing buffers to SD card
	// Set the last char of the message buffer to a newline
	msg_dest[MAX_TEXT_MSG_LENGTH - 1] = '\n';

	// Increment the number of completed log messages in this buffer
	xSemaphoreGive(buffer->msgs_done_semaphore);
	// If last message in buffer, send buffer to queue
	if (msg_num == TEXT_MSGS_PER_BUFFER - 1) {
		if (xQueueSendToBack(full_buffers_queue, &buffer, 0) != pdPASS) {
			// This should never be reached as the queue should have space for all buffers
			log_status.crit_errs++;
			return W_FAILURE;
		}
	}
	// msg_num is guaranteed not to be greater than TEXT_MSGS_PER_BUFFER by condition before write
		
	return W_SUCCESS;
}

w_status_t log_data(log_data_type_t type, const log_data_container_t data) {
	float timestamp = -1.0f;
	timer_get_ms(&timestamp);

	if (xSemaphoreTake(log_data_write_mutex, 10) != pdPASS) {
		log_status.log_write_timeouts++;
		log_status.dropped_msgs++;
		return W_FAILURE;
	}

	log_buffer_t *buffer = &log_data_buffers[current_data_buf_num];

	if (buffer->is_full) {
		xSemaphoreGive(log_data_write_mutex);
		log_status.full_buffer_moments++;
		log_status.dropped_msgs++;
		return W_FAILURE;
	}

	const uint32_t msg_num = buffer->current_msg_num++;

	if (msg_num == DATA_MSGS_PER_BUFFER - 1) {
		buffer->is_full = true;
		current_data_buf_num = (current_data_buf_num + 1) % NUM_DATA_LOG_BUFFERS;
	}

	xSemaphoreGive(log_data_write_mutex);

	if (msg_num >= DATA_MSGS_PER_BUFFER) {
		log_status.invalid_region_moments++;
		log_status.dropped_msgs++;
		return W_FAILURE;
	}

	char *const msg_dest = buffer->data + msg_num * MAX_DATA_MSG_LENGTH;
	uint32_t chars_written = 0;
	if (chars_written + sizeof(type) <= MAX_DATA_MSG_LENGTH) {
		memcpy(msg_dest + chars_written, &type, sizeof(type));
		chars_written += sizeof(type);
	}
	if (chars_written + sizeof(timestamp) <= MAX_DATA_MSG_LENGTH) {
		memcpy(msg_dest + chars_written, &timestamp, sizeof(timestamp));
		chars_written += sizeof(timestamp);
	}
	if (chars_written + sizeof(data) <= MAX_DATA_MSG_LENGTH) {
		memcpy(msg_dest + chars_written, &data, sizeof(data));
		chars_written += sizeof(data);
	}
	// Remaining bytes are zeroed by task after flushing buffers to SD card
	// Set the last char of the message buffer to a newline
	msg_dest[MAX_DATA_MSG_LENGTH - 1] = '\n';

	// Increment the number of completed log messages in this buffer
	xSemaphoreGive(buffer->msgs_done_semaphore);
	// If last message in buffer, send buffer to queue
	if (msg_num == DATA_MSGS_PER_BUFFER - 1) {
		if (xQueueSendToBack(full_buffers_queue, &buffer, 0) != pdPASS) {
			// This should never be reached as the queue should have space for all buffers
			log_status.crit_errs++;
			return W_FAILURE;
		}
	}
	// msg_num is guaranteed not to be greater than DATA_MSGS_PER_BUFFER by condition before write
		
	return W_SUCCESS;
}

void log_task(void *argument) {
	uint32_t run_count = 0;
	uint32_t size = 0;
	w_status_t status = sd_card_file_read(LOG_RUN_COUNT_FILENAME, &run_count, sizeof(run_count), &size);
	if ((status != W_SUCCESS) || (size != sizeof(run_count))) {
		run_count = 0;
	} else {
		run_count++;
	}
	sd_card_file_write(LOG_RUN_COUNT_FILENAME, &run_count, sizeof(run_count), &size);

	char text_log_filename[8 + 4 + 1] = "xxxxxxxx.txt";
	char data_log_filename[8 + 4 + 1] = "xxxxxxxx.bin";
	for (int i = 0; i < 8; i++) {
		uint8_t nibble = (run_count >> (i * 4)) & 0xf;
		char c = nibble + '0';
		if (nibble > 9) {
			c = nibble + 'a';
		}
		text_log_filename[8 - i - 1] = c;
		data_log_filename[8 - i - 1] = c;
	}

	if (sd_card_file_create(text_log_filename) != W_SUCCESS) {
		// TODO: proper error reporting
		while (1) {
			vTaskDelay(portMAX_DELAY);
		}
	}
	if (sd_card_file_create(data_log_filename) != W_SUCCESS) {
		// TODO: proper error reporting
		while (1) {
			vTaskDelay(portMAX_DELAY);
		}
	}

	log_buffer_t *buffer_to_print = NULL;
	for (;;) {
		if (xQueueReceive(full_buffers_queue, &buffer_to_print, 5000) == pdPASS) {
			uint32_t msgs_done = 0;
			uint32_t max_msgs = TEXT_MSGS_PER_BUFFER;
			char *filename = text_log_filename;
			if (!buffer_to_print->is_text) {
				max_msgs = DATA_MSGS_PER_BUFFER;
				filename = data_log_filename;
			}
			while (msgs_done < TEXT_MSGS_PER_BUFFER) {
				if (xSemaphoreTake(buffer_to_print->msgs_done_semaphore, 10) != pdPASS) {
					// TODO: proper error reporting
					while (1) {
						vTaskDelay(portMAX_DELAY);
					}
				} else {
					msgs_done++;
				}
			}
			sd_card_file_write(filename, buffer_to_print->data, LOG_BUFFER_SIZE, &size);
			log_reset_buffer(buffer_to_print);
		} else {
			log_status.no_full_buf_moments++;
		}
	}
}
