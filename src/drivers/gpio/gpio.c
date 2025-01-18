/**
 * GPIO module
 * Driver for GPIO pins
 */
#include <stdbool.h>

#include "stm32h7xx_hal.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "drivers/gpio/gpio.h"

/**
 * Private status trackers
 */
typedef struct {
    bool is_init; // Whether module is initialized
    uint32_t accesses; // # of successful r/w
    uint32_t access_fails; // # of failed r/w
    bool err; // Whether module is currently in err state for any reason
} gpio_module_status;

/**
 * Private gpio pin data for 1 pin
 */
typedef struct {
    GPIO_TypeDef *port; // HAL GPIO port
    uint16_t pin; // HAL GPIO pin bit mask
    SemaphoreHandle_t access_mutex; // access mutex
} gpio_pin_data_t;

/**
 * Map gpio pins enums to their hardware data
 */
gpio_pin_data_t gpio_map[GPIO_PIN_COUNT] = {
    [GPIO_PIN_RED_LED] = {.port = GPIOE, .pin = GPIO_PIN_9},
    [GPIO_PIN_GREEN_LED] = {.port = GPIOE, .pin = GPIO_PIN_10},
    [GPIO_PIN_BLUE_LED] = {.port = GPIOE, .pin = GPIO_PIN_11},
};

/**
 * Initialize gpio module. Can be run before scheduler start
 */
w_status_t gpio_init() {
    w_status_t status = W_SUCCESS;

    return status;
}

/**
 * Read the current level of `pin` into `level`.
 * Block for up to `timeout` ms.
 */
w_status_t gpio_read(gpio_pin_t pin, gpio_level_t *level, uint32_t timeout) {
    w_status_t status = W_SUCCESS;

    return status;
}

/**
 * Set `pin` to `level`. Block for up to `timeout` ms.
 */
w_status_t gpio_write(gpio_pin_t pin, gpio_level_t level, uint32_t timeout) {
    w_status_t status = W_SUCCESS;

    return status;
}

/**
 * Toggle `pin` between high/low from its current level.
 * Block for up to `timeout` ms.
 */
w_status_t gpio_toggle(gpio_pin_t pin, uint32_t timeout) {
    w_status_t status = W_SUCCESS;

    return status;
}
