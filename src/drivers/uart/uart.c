#include "drivers/uart/uart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include <string.h>

// Define maximum allowed length for a single UART message (256 Bytes)
#define MAX_MSG_LENGTH 256 // Maximum buffer size for UART messages

/* Handle structure for each UART channel. Contains all state needed for
 * double-buffered message reception. The double buffer design allows one
 * buffer to be filled while the other is being read, preventing data loss
 * during back-to-back message reception. */
typedef struct {
    UART_HandleTypeDef *huart; /* HAL UART handle from CubeMX */
    uint32_t timeout_ms; /* Default timeout for operations */
    uart_msg_t rx_msgs[2]; /* Two buffers for double-buffering */
    uint8_t curr_write_ind; /* Which buffer (0/1) is receiving */
    QueueHandle_t avail_msgs; /* Queue for completed messages */
} uart_handle_t;

/* Array of UART handles - one per channel.
 * Static allocation ensures fixed memory usage. */
static uart_handle_t s_uart_handles[UART_CHANNEL_COUNT];

/* Forward declaration for IDLE event handler.
 * Keeps main UART callback clean and simple. */
static void handle_idle_event(UART_HandleTypeDef *huart, uint16_t size);

/* Initialize a UART channel for double-buffered reception.
 * Must be called before RTOS starts since it creates RTOS primitives. */
w_status_t uart_init(uart_channel_t channel, UART_HandleTypeDef *huart, uint32_t timeout_ms) {
    /* Validate parameters before doing any initialization */
    if (channel >= UART_CHANNEL_COUNT || huart == NULL) {
        return W_INVALID_PARAM;
    }

    /* Get handle for this channel and clear it to known state */
    uart_handle_t *handle = &s_uart_handles[channel];
    memset(handle, 0, sizeof(*handle));
    handle->huart = huart;
    handle->timeout_ms = timeout_ms;

    /* Initialize both reception buffers to empty and not busy.
     * busy flag helps prevent buffer overwrite if reader is slow. */
    for (int i = 0; i < 2; i++) {
        handle->rx_msgs[i].len = 0;
        handle->rx_msgs[i].busy = false;
    }

    /* Start with buffer 0 for initial reception */
    handle->curr_write_ind = 0;

    /* Create queue for passing complete messages to readers.
     * Single slot queue - we only keep most recent message. */
    handle->avail_msgs = xQueueCreate(1, sizeof(uart_msg_t));
    if (handle->avail_msgs == NULL) {
        return W_FAILURE;
    }

    /* Start initial reception on first buffer.
     * Using IDLE detection for automatic message framing. */
    if (HAL_UARTEx_ReceiveToIdle_IT(huart, handle->rx_msgs[0].data, MAX_MSG_LEN) != HAL_OK) {
        return W_IO_ERROR;
    }

    return W_SUCCESS;
}

/* Read latest complete message from specified UART channel.
 * Blocks until message arrives or timeout occurs. */
w_status_t
uart_read(uart_channel_t channel, uint8_t *buffer, uint16_t *length, uint32_t timeout_ms) {
    /* Validate all parameters before proceeding */
    if (channel >= UART_CHANNEL_COUNT || buffer == NULL || length == NULL) {
        return W_INVALID_PARAM;
    }

    uart_handle_t *handle = &s_uart_handles[channel];
    uart_msg_t msg;

    /* Wait for message to become available.
     * xQueueReceive blocks until message arrives or timeout. */
    if (xQueueReceive(handle->avail_msgs, &msg, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        *length = 0;
        return W_IO_TIMEOUT;
    }

    /* Copy message to user's buffer. Length check prevents buffer overflow,
     * though MAX_MSG_LEN should already enforce this. */
    uint16_t msg_len = (uint16_t)msg.len;
    if (msg_len > MAX_MSG_LEN) {
        msg_len = MAX_MSG_LEN;
    }
    memcpy(buffer, msg.data, msg_len);
    *length = msg_len;

    return W_SUCCESS;
}

/* HAL callback for UART IDLE line detection.
 * Called from interrupt context when message completes. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    /* Delegate to handler to keep ISR simple */
    handle_idle_event(huart, size);
}

/* Process a completed message reception.
 * Called from ISR context so uses _FromISR RTOS functions. */
static void handle_idle_event(UART_HandleTypeDef *huart, uint16_t size) {
    BaseType_t higher_priority_task_woken = pdFALSE;

    /* Find which UART channel triggered this event */
    uart_channel_t ch;
    for (ch = 0; ch < UART_CHANNEL_COUNT; ch++) {
        if (s_uart_handles[ch].huart == huart) {
            break;
        }
    }
    if (ch >= UART_CHANNEL_COUNT) {
        return; /* Not one of our UARTs */
    }

    uart_handle_t *handle = &s_uart_handles[ch];
    uint8_t w_ind = handle->curr_write_ind;
    uart_msg_t *msg = &handle->rx_msgs[w_ind];

    /* Mark current buffer's message as complete */
    msg->len = size;
    msg->busy = true;

    /* Make message available to readers. Uses overwrite version
     * so newest message is never blocked by old unread message. */
    xQueueOverwriteFromISR(handle->avail_msgs, msg, &higher_priority_task_woken);

    /* Try to switch to other buffer for next reception.
     * Only switch if other buffer isn't still busy from last time. */
    uint8_t next_ind = (w_ind + 1U) % 2U;
    if (!handle->rx_msgs[next_ind].busy) {
        handle->curr_write_ind = next_ind;
    }

    /* Start new reception on whichever buffer we chose.
     * If we couldn't switch buffers, we'll overwrite current one. */
    uart_msg_t *next_msg = &handle->rx_msgs[handle->curr_write_ind];
    next_msg->len = 0; /* Reset length for new message */
    HAL_UARTEx_ReceiveToIdle_IT(huart, next_msg->data, MAX_MSG_LEN);

    /* Request context switch if higher priority task woken */
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/* Handle UART error conditions.
 * Called from ISR context when UART detects an error. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    BaseType_t higher_priority_task_woken = pdFALSE;

    /* Find which channel had the error */
    uart_channel_t channel;
    for (channel = 0; channel < UART_CHANNEL_COUNT; channel++) {
        if (s_uart_handles[channel].huart == huart) {
            break;
        }
    }
    if (channel >= UART_CHANNEL_COUNT) {
        return; /* Not our UART */
    }

    uart_handle_t *handle = &s_uart_handles[channel];

    /* Check all possible error flags */
    uint32_t error = HAL_UART_GetError(huart);

    if (error & HAL_UART_ERROR_PE) {
        /* Parity error indicates possible data corruption */
    }
    if (error & HAL_UART_ERROR_NE) {
        /* Noise error might resolve with retry */
    }
    if (error & HAL_UART_ERROR_FE) {
        /* Framing error means we lost synchronization */
    }
    if (error & HAL_UART_ERROR_ORE) {
        /* Overrun means we lost data, need to resync */
    }

    /* Clear error flags for next reception */
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* Restart reception to recover from error */
    uart_msg_t *curr_msg = &handle->rx_msgs[handle->curr_write_ind];
    curr_msg->len = 0;
    HAL_UARTEx_ReceiveToIdle_IT(huart, curr_msg->data, MAX_MSG_LEN);

    portYIELD_FROM_ISR(higher_priority_task_woken);
}