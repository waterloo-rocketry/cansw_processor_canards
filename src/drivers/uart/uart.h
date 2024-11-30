#ifndef UART_H
#define UART_H

#include "rocketlib/include/common.h"
#include <stdint.h>

typedef enum
{
	UART_IMU;		   // Movella IMU
	UART_DEBUG_SERIAL; // debugger serial

}
uart_device_t;

// Must be called before RTOS scheduler starts
w_status_t uart_init(uart_device_t device, uint32_t baud, uint32_t timeout_ms);

w_status_t uart_write(uart_device_t device, const uint8_t *data, uint8_t len);

#endif // UART_H
