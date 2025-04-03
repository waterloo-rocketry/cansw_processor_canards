# Hardware-in-the-Loop (HIL) module
add_library(hil
    hil.c
    simulator.c # Include the simulator implementation
)

target_include_directories(hil
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/src/third_party             # For rocketlib
        ${CMAKE_SOURCE_DIR}/src/third_party/canlib    # For canlib.h needed via estimator->can_handler
        # CMSIS Includes (needed for arm_math.h via estimator->controller->gain_scheduling)
        ${CMAKE_SOURCE_DIR}/src/third_party/CMSIS-DSP/Include
        ${CMAKE_SOURCE_DIR}/src/third_party/CMSIS-DSP/PrivateInclude 
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include  # Core CMSIS headers
)

# Add compile definitions needed by included headers
target_compile_definitions(hil PUBLIC
    BOARD_INST_UNIQUE_ID=BOARD_INST_ID_GENERIC
    BOARD_TYPE_UNIQUE_ID=BOARD_TYPE_ID_PROCESSOR
)

target_link_libraries(hil
    PRIVATE
        stm32cubemx # Link against the target providing HAL/FreeRTOS core includes/libs
) 