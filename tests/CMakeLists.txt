cmake_minimum_required(VERSION 3.22)

# Add coverage flags globally
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")

# Enable faster linking with the gold linker if available
# This is commonly supported in CI environments but we'll check first
if(UNIX AND NOT APPLE)
  include(CheckLinkerFlag)
  check_linker_flag(CXX "-fuse-ld=gold" GOLD_LINKER_SUPPORTED)
  if(GOLD_LINKER_SUPPORTED)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold")
  endif()
endif()

add_compile_definitions(GTEST)


# Define paths
set(MOCKS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mocks)
set(FFF_DIR ${MOCKS_DIR}/fff)
set(UNIT_TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/unit)
# Full path to the file
set(PROC_SRC_DIR "${CMAKE_SOURCE_DIR}/src")

# add ctest
include(CTest)
# Add GoogleTest
add_subdirectory(external/googletest)
enable_testing()
include(GoogleTest)
add_compile_definitions(GTEST)

# helper to create a test executable
# usage: `add_test_group(${test_name} ${src_files})`
# src_files must be a list of the test src file(s) and the proc src file(s) being tested
macro(add_test_group TEST_NAME)
    add_executable(${TEST_NAME} ${ARGN})  # Expands all additional arguments as a list
    # all tests get the same common include paths and common libraries
    target_include_directories(${TEST_NAME} PRIVATE ${TEST_COMMON_INCS})
    target_link_libraries(${TEST_NAME} PRIVATE ${TEST_COMMON_LIBS})
    # canlib needs this so give it to everyone ig
    target_compile_definitions(${TEST_NAME} PRIVATE
        BOARD_INST_UNIQUE_ID=0x01
        BOARD_TYPE_UNIQUE_ID=0X06
    )
    # register the tests with ctest (via this gtest helper)
    gtest_discover_tests(${TEST_NAME})
    # the coverage report depends on this test being built first
    add_dependencies(cov ${TEST_NAME})
endmacro()

# Make tests automatically generate coverage report after running
add_custom_target(cov ALL
    # first, run all the test groups
    COMMAND ctest --output-on-failure
    # then, generate the coverage report
    COMMAND lcov -b ${CMAKE_BINARY_DIR} -capture --directory ${CMAKE_BINARY_DIR} --output-file src_coverage.info
    # only include the 'unit' subdirs in the coverage report cuz those are the proc source files we're testing
    # (ie, idc how much of canlib we tested cuz thats not our code)
    COMMAND lcov -b ${CMAKE_BINARY_DIR} --extract src_coverage.info ${PROC_SRC_DIR}/drivers/*/* ${PROC_SRC_DIR}/application/*/* --output-file src_coverage_filtered.info
    # generate the html coverage report
    # TODO: make this display for github pipeline in terminal? idk
    COMMAND genhtml src_coverage_filtered.info --output-directory coverage_report

    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running tests and generating coverage report (only including 'unit' subdirectories)..."
)

# add cmsis dsp library. Do this manually instead of using the cmsis-dsp
# CMakeLists.txt to avoid building unused cmsisdsp modules which would be slow and fat (derogatory)
set(CMSISDSP_DIR ${PROC_SRC_DIR}/third_party/CMSIS-DSP)
add_library(CMSISDSP STATIC
    ${CMSISDSP_DIR}/Source/BasicMathFunctions/BasicMathFunctions.c 
    ${CMSISDSP_DIR}/Source/InterpolationFunctions/InterpolationFunctions.c 
    ${CMSISDSP_DIR}/Source/MatrixFunctions/MatrixFunctions.c 
    ${CMSISDSP_DIR}/Source/QuaternionMathFunctions/QuaternionMathFunctions.c 
    ${CMSISDSP_DIR}/Source/FastMathFunctions/FastMathFunctions.c 
)
# the include paths needed by cmsis-dsp itself
target_include_directories(CMSISDSP PUBLIC "${CMSISDSP_DIR}/Include")
target_include_directories(CMSISDSP PUBLIC "${CMSISDSP_DIR}/PrivateInclude")
# cmsis-dsp also requires the core cmsis files. use the cmsis folder provided by cubemx
target_include_directories(CMSISDSP PUBLIC "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include")


# define the common include paths that all tests may use
set(TEST_COMMON_INCS
    ${MOCKS_DIR}
    ${FFF_DIR}
    ${PROC_SRC_DIR}
    ${PROC_SRC_DIR}/third_party
    ${PROC_SRC_DIR}/third_party/canlib
)

# Add FFF as a header-only library
add_library(fff INTERFACE)
target_include_directories(fff INTERFACE ${FFF_DIR})

# Add mocks as a static lib
file(GLOB MOCK_SOURCES "${MOCKS_DIR}/*.c")
add_library(mocks STATIC ${MOCK_SOURCES})
target_include_directories(mocks PUBLIC ${TEST_COMMON_INCS})

# define the common libs that all tests may use
set(TEST_COMMON_LIBS
    fff           # Fake Function Framework
    mocks         # our mocks
    gtest_main    # GoogleTest
    CMSISDSP
)

###############################################################################
# Add test groups one by one using the helper macro `add_test_group(${test_name} ${src_files})`
# src_files is the list of the test src file(s) and the proc src file(s) being tested
# The list of src_files must be in quotations since it's technically 1 argument
###############################################################################
add_test_group(test_example
    # test file(s)
    ${UNIT_TESTS_DIR}/example_test.cpp
    # src file(s)
    # (example test doesn't actually test any proc source files)
)


add_test_group(test_gpio
    # test file(s)
    ${UNIT_TESTS_DIR}/gpio_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/drivers/gpio/gpio.c
)

add_test_group(test_uart
    # test file(s)
    ${UNIT_TESTS_DIR}/uart_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/drivers/uart/uart.c
)

add_test_group(test_estimator_math
    # test file(s)
    ${UNIT_TESTS_DIR}/estimator_lib_test.cpp
    # TODO: add estimator math source files when they exist
)

add_test_group(test_i2c
    # test file(s)
    ${UNIT_TESTS_DIR}/i2c_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/drivers/i2c/i2c.c
)

add_test_group(test_timer
    # test file(s)
    ${UNIT_TESTS_DIR}/timer_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/drivers/timer/timer.c
)



add_test_group(test_imu_handler
    # test file(s)
    ${UNIT_TESTS_DIR}/imu_handler_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/imu_handler/imu_handler.c
)


add_test_group(test_flight_phase
    # test files
    ${UNIT_TESTS_DIR}/flight_phase_test.cpp
    # src files
    ${PROC_SRC_DIR}/application/flight_phase/flight_phase.c
)

add_test_group(test_altimu
    # test file(s)
    ${UNIT_TESTS_DIR}/altimu_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/drivers/altimu-10/altimu-10.c
)


