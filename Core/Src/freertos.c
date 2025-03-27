/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h7xx_hal.h"

#include "application/can_handler/can_handler.h"
#include "application/controller/controller.h"
#include "application/estimator/estimator.h"
#include "application/flight_phase/flight_phase.h"
#include "application/health_checks/health_checks.h"
#include "application/imu_handler/imu_handler.h"
#include "application/logger/log.h"
#include "drivers/adc/adc.h"
#include "drivers/gpio/gpio.h"
#include "drivers/i2c/i2c.h"
#include "drivers/movella/movella.h"
#include "drivers/sd_card/sd_card.h"
#include "drivers/timer/timer.h"
#include "drivers/uart/uart.h"

#include "rocketlib/include/common.h"
#include "third_party/printf/printf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// External HAL handles
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c4;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart8;
extern FDCAN_HandleTypeDef hfdcan1;
extern ADC_HandleTypeDef hadc1;

// Task handles
TaskHandle_t log_task_handle = NULL;
TaskHandle_t estimator_task_handle = NULL;
TaskHandle_t can_handler_handle_tx = NULL;
TaskHandle_t can_handler_handle_rx = NULL;
TaskHandle_t health_checks_task_handle = NULL;
TaskHandle_t controller_task_handle = NULL;
TaskHandle_t flight_phase_task_handle = NULL;
TaskHandle_t imu_handler_task_handle = NULL;
TaskHandle_t movella_task_handle = NULL;

// flight phase must have highest priority to preempt everything else
const uint32_t flight_phase_task_priority = configMAX_PRIORITIES - 1;
// TODO: replace with actual priorities once determined. for now just make all same priority
const uint32_t log_task_priority = configMAX_PRIORITIES - 5;
const uint32_t estimator_task_priority = configMAX_PRIORITIES - 5;
const uint32_t controller_task_priority = configMAX_PRIORITIES - 5;
const uint32_t can_handler_rx_priority = configMAX_PRIORITIES - 5;
const uint32_t can_handler_tx_priority = configMAX_PRIORITIES - 5;
const uint32_t health_checks_task_priority = configMAX_PRIORITIES - 5;
const uint32_t imu_handler_task_priority = configMAX_PRIORITIES - 5;
const uint32_t movella_task_priority = configMAX_PRIORITIES - 5;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void) {}

extern volatile unsigned long ulHighFrequencyTimerTicks;
__weak unsigned long getRunTimeCounterValue(void) {
    return ulHighFrequencyTimerTicks;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName) {
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
    called if a stack overflow is detected. */
    while (1) {}
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void) {
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created. It is also called by various parts of the
    demo application. If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    while (1) {}
}
/* USER CODE END 5 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * This task simply blinks all 3 leds at 1hz
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
    /* USER CODE BEGIN StartDefaultTask */

    // ALL CANARD MODULE INITIALIZATION GOES HERE -------------------------
    // TODO: ideally move this into a separate file instead of trying to abide by
    // stupid stm32 hal file structure
    w_status_t status = W_SUCCESS;

    status |= gpio_init();
    status |= i2c_init(I2C_BUS_2, &hi2c2, 0);
    status |= i2c_init(I2C_BUS_4, &hi2c4, 0);
    status |= uart_init(UART_DEBUG_SERIAL, &huart4, 100);
    status |= uart_init(UART_MOVELLA, &huart8, 100);
    status |= adc_init(&hadc1);
    status |= flight_phase_init();
    status |= can_handler_init(&hfdcan1);
    status |= controller_init();
    status |= imu_handler_init();

    if (status != W_SUCCESS) {
        // TODO: handle init failure. for now get stuck here for debugging purposes
        while (1) {
            /* spin */
        }
    }

    BaseType_t task_init_status = xTaskCreate(
        flight_phase_task,
        "flight phase",
        512,
        NULL,
        flight_phase_task_priority,
        &flight_phase_task_handle
    );
    task_init_status &= xTaskCreate(
        imu_handler_task, "imu handler", 512, NULL, log_task_priority, &log_task_handle
    );
    task_init_status &= xTaskCreate(
        can_handler_task_rx,
        "can handler rx",
        512,
        NULL,
        can_handler_rx_priority,
        &can_handler_handle_rx
    );
    task_init_status &= xTaskCreate(
        can_handler_task_tx,
        "can handler tx",
        512,
        NULL,
        can_handler_tx_priority,
        &can_handler_handle_tx
    );
    task_init_status &= xTaskCreate(
        movella_task, "movella", 2560, NULL, movella_task_priority, &movella_task_handle
    );

    if (task_init_status != pdPASS) {
        while (1) {
            // error
        }
    }

    // perform blinky
    for (;;) {
        w_status_t status = W_SUCCESS;

        // Toggle all 3 leds
        status |= gpio_toggle(GPIO_PIN_RED_LED, 0);
        status |= gpio_toggle(GPIO_PIN_GREEN_LED, 0);
        status |= gpio_toggle(GPIO_PIN_BLUE_LED, 0);

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second

        if (status != W_SUCCESS) {
            // TODO: handle failure
        }
    }
    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

