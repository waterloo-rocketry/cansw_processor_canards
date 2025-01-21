cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME processor_board)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})
add_compile_options(-fdebug-prefix-map=${CMAKE_SOURCE_DIR}=../..)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory("cmake/stm32cubemx")

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
file(GLOB_RECURSE C_SOURCES "src/application/*.c"
    "src/drivers/*.c")

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${C_SOURCES}
    src/drivers/i2c/i2c_handles.c
    # only add lib files used by this project
    src/third_party/canlib/message/msg_actuator.c
    src/third_party/canlib/message/msg_common.c
    src/third_party/canlib/message/msg_general.c
    src/third_party/canlib/message/msg_gps.c
    src/third_party/canlib/message/msg_recovery.c
    src/third_party/canlib/message/msg_sensor.c
    src/third_party/canlib/message/msg_state_est.c
    src/third_party/canlib/stm32h7/stm32h7_can.c
    src/third_party/canlib/util/can_rcv_buffer.c
    src/third_party/canlib/util/can_tx_buffer.c
    src/third_party/canlib/util/safe_ring_buffer.c
    src/third_party/canlib/util/timing_util.c
    src/third_party/printf/printf.c
    
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/cmake/stm32cubemx/Core/Inc"           # Contains stm32h7xx_hal_conf.h
    ${CMAKE_SOURCE_DIR}/cmake/stm32cubemx/Drivers/STM32H7xx_HAL_Driver/Inc"  # HAL driver headers
    src
    src/third_party
    src/third_party/canlib
    src/third_party/printf
    src/drivers/i2c
    
)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    "-include" "${CMAKE_SOURCE_DIR}/cmake/stm32cubemx/Core/Inc/stm32h7xx_hal_conf.h"
)


# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    BOARD_INST_UNIQUE_ID=0x01
    BOARD_TYPE_UNIQUE_ID=0X06
    
)



# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
