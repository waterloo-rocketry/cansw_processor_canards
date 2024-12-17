/**
 * GPIO module
 * Driver for GPIO pins
 */
#ifndef GPIO_H
#define GPIO_H

#include "common.h"

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
 * Read the current level of `pin` into `level`. Non-blocking. Not thread-safe
 * if multiple threads interact with the same pin.
 */
w_status_t gpio_read(gpio_pin_t pin, gpio_level_t *level);

/**
 * Set `pin` to be `level`. Non-blocking. Not thread-safe
 * if multiple threads interact with the same pin.
 */
w_status_t gpio_write(gpio_pin_t pin, gpio_level_t level);

/**
 * Toggle `pin` between high/low from its current level. Non-blocking. Not thread-safe
 * if multiple threads interact with the same pin.
 */
w_status_t gpio_toggle(gpio_pin_t pin);

#endif // GPIO_H
