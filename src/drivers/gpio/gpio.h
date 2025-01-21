/**
 * GPIO module
 * Driver for GPIO pins
 */
#ifndef GPIO_H
#define GPIO_H

#include "rocketlib/include/common.h"

/**
 * Enum representing GPIO pin level
 */
typedef enum {
    GPIO_LEVEL_LOW,
    GPIO_LEVEL_HIGH
} gpio_level_t;

/**
 * Enum representing a connected GPIO pin
 */
typedef enum {
    GPIO_PIN_RED_LED,
    GPIO_PIN_GREEN_LED,
    GPIO_PIN_BLUE_LED,
    GPIO_PIN_COUNT // Enum max value
} gpio_pin_t;

/**
 * Read the current level of `pin` into `level`.
 * Block for up to `timeout` ms.
 */
w_status_t gpio_read(gpio_pin_t pin, gpio_level_t *level, uint32_t timeout);

/**
 * Set `pin` to `level`. Block for up to `timeout` ms.
 */
w_status_t gpio_write(gpio_pin_t pin, gpio_level_t level, uint32_t timeout);

/**
 * Toggle `pin` between high/low from its current level.
 * Block for up to `timeout` ms.
 */
w_status_t gpio_toggle(gpio_pin_t pin, uint32_t timeout);

#endif // GPIO_H
