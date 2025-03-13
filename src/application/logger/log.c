#include <stdbool.h>

#include "queue.h"
#include "semphr.h"
#include "rocketlib/include/common.h"
#include "application/logger/log.h"

typedef struct {
	bool is_text;
	bool is_full;
	unsigned int current_msg_num;
	SemaphoreHandle_t current_msg_num_mutex;
	SemaphoreHandle_t msgs_done_semaphore;
	char data[LOG_BUFFER_SIZE];
} log_buffer_t;

static log_buffer_t log_text_buffers[NUM_TEXT_LOG_BUFFERS];
static log_buffer_t log_data_buffers[NUM_DATA_LOG_BUFFERS];
static unsigned int current_text_buf_num;
static unsigned int current_data_buf_num;
static SemaphoreHandle_t log_text_write_mutex;
static SemaphoreHandle_t log_data_write_mutex;
static QueueHandle_t full_buffers_queue;

static unsigned int dropped_msgs;
static unsigned int full_buffer_moments;
static unsigned int log_write_timeouts;
static unsigned int invalid_region_moments;
static unsigned int crit_errs;
static unsigned int no_full_buf_moments;

void log_init(void) {
	// TODO
}

w_status_t log_text(const char *source, const char *format, ...) {
	TickType_t timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

	if (xSemaphoreTake(log_text_write_mutex, 10) != pdPASS) {
		log_write_timeouts++;
		dropped_msgs++;
		return W_FAILURE;
	}

	log_buffer_t *buffer = &log_text_buffers[current_text_buf_num];

	if (buffer->is_full) {
		xSemaphoreGive(log_text_write_mutex);
		full_buffer_moments++;
		dropped_msgs++;
		return W_FAILURE;
	}

	unsigned int msg_num = buffer->current_msg_num++;

	if (msg_num == TEXT_MSGS_PER_BUFFER - 1) {
		buffer->is_full = true;
		current_text_buf_num = (current_text_buf_num + 1) % NUM_TEXT_LOG_BUFFERS;
	}

	xSemaphoreGive(log_text_write_mutex);

	if (msg_num >= TEXT_MSGS_PER_BUFFER) {
		invalid_region_moments++;
		dropped_msgs++;
		return W_FAILURE;
	}

	char *msg_dest = buffer->data + msg_num * MAX_TEXT_MSG_LENGTH;
	// TODO
}

w_status_t log_data(log_data_type_t type, const log_data_container_t data) {
	TickType_t timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

	if (xSemaphoreTake(log_data_write_mutex, 10) != pdPASS) {
		log_write_timeouts++;
		dropped_msgs++;
		return W_FAILURE;
	}

	log_buffer_t *buffer = &log_data_buffers[current_data_buf_num];

	if (buffer->is_full) {
		xSemaphoreGive(log_data_write_mutex);
		full_buffer_moments++;
		dropped_msgs++;
		return W_FAILURE;
	}

	unsigned int msg_num = buffer->current_msg_num++;

	if (msg_num == DATA_MSGS_PER_BUFFER - 1) {
		buffer->is_full = true;
		current_data_buf_num = (current_data_buf_num + 1) % NUM_DATA_LOG_BUFFERS;
	}

	xSemaphoreGive(log_data_write_mutex);

	if (msg_num >= DATA_MSGS_PER_BUFFER) {
		invalid_region_moments++;
		dropped_msgs++;
		return W_FAILURE;
	}

	char *msg_dest = buffer->data + msg_num * MAX_DATA_MSG_LENGTH;
	// TODO
}

void log_task(void *argument) {
	log_buffer_t *buffer_to_print = NULL;
	// TODO: files
	for (;;) {
		if (xQueueReceive(full_buffers_queue, &buffer_to_print, 5000) == pdPASS) {
			// TODO: write
			buffer_to_print->current_msg_num = 0;
			buffer_to_print->is_full = false;
		} else {
			no_full_buf_moments++;
		}
	}
}
