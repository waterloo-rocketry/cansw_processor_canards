# Hardware-in-the-Loop (HIL) module
add_library(hil
    hil.c
    simulator.c # Include the simulator implementation
)

target_include_directories(hil
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(hil
    PUBLIC
        common      # For w_status_t and other common types
        FreeRTOS    # For tick manipulation
        estimator   # Simulator calls estimator functions
    PRIVATE
        HAL         # For HAL types like UART_HandleTypeDef if needed
) 