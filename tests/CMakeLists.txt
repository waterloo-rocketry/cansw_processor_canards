cmake_minimum_required(VERSION 3.14)
project(unit_tests C CXX)

# Set the C++ standard.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Define Paths (adjust these paths for your project structure)
set(MOCKS_DIR "${CMAKE_SOURCE_DIR}/mocks")            # Directory for your mock headers & sources.
set(FFF_DIR   "${MOCKS_DIR}/fff")                        # Directory containing FFF headers (including fff.h).
set(UNIT_TESTS_DIR "${CMAKE_SOURCE_DIR}/unit")          # Directory containing your test .cpp files.
set(SRC_DIR   "${CMAKE_SOURCE_DIR}/../src")             # Your project source files.

# If your tests require rocketlib headers, set its include directory.
set(ROCKETLIB_DIR "${SRC_DIR}/rocketlib/include")       # Adjust if needed.

# ---------------------------------------------------------------------------
# Disable FFF argument history (to avoid errors about custom_fake_seq, etc.)
add_compile_definitions(FFF_ARG_HISTORY_LEN=0)

# ---------------------------------------------------------------------------
# Add FFF as a header-only library.
add_library(fff INTERFACE)
target_include_directories(fff INTERFACE "${FFF_DIR}")

# ---------------------------------------------------------------------------
# Add the Mocks Library.
# Gather all C source files in the mocks directory.
file(GLOB MOCK_SOURCES "${MOCKS_DIR}/*.c")

# If duplicate files exist (e.g. a separate mock_i2c_hal.c), filter them out.
list(FILTER MOCK_SOURCES EXCLUDE REGEX ".*mock_i2c_hal(\\.c)?$")

# Then ensure that the consolidated HAL mock file and the dummy I2C handles file are added.
list(APPEND MOCK_SOURCES "${MOCKS_DIR}/stm32h7xx_hal.c")
list(APPEND MOCK_SOURCES "${MOCKS_DIR}/mock_i2c_handles.c")  # This file should define hi2c1, hi2c2, hi2c3

if(NOT MOCK_SOURCES)
    message(FATAL_ERROR "No source files found in ${MOCKS_DIR}")
endif()
add_library(mocks STATIC ${MOCK_SOURCES})
target_include_directories(mocks PUBLIC 
    "${MOCKS_DIR}"     # This folder should contain your mock version of stm32h7xx_hal.h.
    "${FFF_DIR}"       # FFF headers.
    "${SRC_DIR}"       
)

# ---------------------------------------------------------------------------
# Build the Project Source Library (proc_src)
# Gather source files from application and drivers.
file(GLOB_RECURSE APPLICATION_FILES "${SRC_DIR}/application/*.c")
file(GLOB_RECURSE DRIVERS_FILES "${SRC_DIR}/drivers/*.c")
set(SRC_FILES ${APPLICATION_FILES} ${DRIVERS_FILES})
if(NOT SRC_FILES)
    message(FATAL_ERROR "No source files found in application or drivers directories")
endif()
add_library(proc_src STATIC ${SRC_FILES})
target_include_directories(proc_src PUBLIC
    "${SRC_DIR}/third_party"
    "${SRC_DIR}"
    "${MOCKS_DIR}"       # Force using the mock HAL header.
    "${FFF_DIR}"         # Ensure fff.h is found.
)

# ---------------------------------------------------------------------------
# Gather Test Files.
file(GLOB TEST_FILES "${UNIT_TESTS_DIR}/*.cpp")
if(NOT TEST_FILES)
    message(FATAL_ERROR "No test files found in ${UNIT_TESTS_DIR}")
endif()

# ---------------------------------------------------------------------------
# Create the Test Executable.
add_executable(unit_tests ${TEST_FILES})
# Example snippet from your CMakeLists.txt
target_include_directories(unit_tests PRIVATE
    ${MOCKS_DIR}                     # This should allow the compiler to find "stm32h7xx_hal_i2c.h" and "mock_freertos.h"
    ${FFF_DIR}                       # To pick up fff.h in case FFF is used by your mocks
    ${SRC_DIR}/drivers               # To find i2c.h (if i2c.h is in src/drivers/i2c/ it will be found)
    ${SRC_DIR}/drivers/i2c           # If your project structure requires this additional path
    ${SRC_DIR}/rocketlib/include     # If your tests include rocketlib files (adjust or remove if not needed)
    ${SRC_DIR}/application           # Any other project include folders
    ${SRC_DIR}                       # General project sources
    ${SRC_DIR}/third_party           # Third‑party libraries
)


# ---------------------------------------------------------------------------
# Link Libraries to the Test Executable.
target_link_libraries(unit_tests PRIVATE
    fff           # FFF (header-only)
    mocks         # Mock library
    proc_src      # Your source code library
    gtest_main    # GoogleTest main (assumed provided by gtest subdirectory)
)

# ---------------------------------------------------------------------------
# Add GoogleTest as a Subdirectory.
add_subdirectory(external/googletest)

enable_testing()
include(GoogleTest)
gtest_discover_tests(unit_tests)
