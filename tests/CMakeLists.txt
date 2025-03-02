cmake_minimum_required(VERSION 3.22)

# Add coverage flags globally
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")

# Define paths
set(MOCKS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mocks)
set(FFF_DIR ${MOCKS_DIR}/fff)
set(UNIT_TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/unit)
set(PROC_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)  

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
    target_include_directories(${TEST_NAME} PRIVATE ${TEST_COMMON_INCS})
    target_link_libraries(${TEST_NAME} PRIVATE ${TEST_COMMON_LIBS})
    gtest_discover_tests(${TEST_NAME})
endmacro()

# Make tests automatically generate coverage report after running
# add_custom_command(TARGET test_all POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E env GTEST_OUTPUT=xml:test-results/ $<TARGET_FILE:tests>

#     # Clean and run coverage for only our src files
#     COMMAND lcov --capture --directory ${CMAKE_BINARY_DIR}/tests/CMakeFiles/proc_src.dir --output-file src_coverage.info
#     COMMAND genhtml src_coverage.info --output-directory coverage_report

#     WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
#     COMMENT "Running tests and generating coverage report..."
# )

# add cmsis dsp library
set(CMSISCORE ${PROC_SRC_DIR}/../Drivers/CMSIS)
add_subdirectory(${PROC_SRC_DIR}/third_party/CMSIS-DSP/Source bin_dsp)

# Add FFF as a header-only library
add_library(fff INTERFACE)
target_include_directories(fff INTERFACE ${FFF_DIR})

# Add mocks as a static lib
file(GLOB MOCK_SOURCES "${MOCKS_DIR}/*.c")
add_library(mocks STATIC ${MOCK_SOURCES})
target_include_directories(mocks PUBLIC ${MOCKS_DIR} ${FFF_DIR} ${PROC_SRC_DIR})

# define the common include paths that all tests may use
set(TEST_COMMON_INCS
    ${MOCKS_DIR}
    ${FFF_DIR}
    ${PROC_SRC_DIR}
    ${PROC_SRC_DIR}/third_party
    ${PROC_SRC_DIR}/third_party/canlib
)

# define the common libs that all tests may use
set(TEST_COMMON_LIBS
    fff           # Fake Function Framework
    mocks         # Mock library
    gtest_main    # GoogleTest main function
    CMSISDSP
)

###############################################################################
# Add test groups one by one using the helper macro `add_test_group(${test_name} ${src_files})`
# src_files is the list of the test src file(s) and the proc src file(s) being tested
# The list of src_files must be in quotations since it's technically 1 argument
###############################################################################
add_test_group(test_gpio
    ${UNIT_TESTS_DIR}/gpio_test.cpp
    ${UNIT_TESTS_DIR}/test_runner.cpp
    ${PROC_SRC_DIR}/drivers/gpio/gpio.c
)

add_test_group(test_uart
    ${UNIT_TESTS_DIR}/uart_test.cpp
    ${UNIT_TESTS_DIR}/test_runner.cpp
    ${PROC_SRC_DIR}/drivers/uart/uart.c
)

add_test_group(test_estimator_math
    ${UNIT_TESTS_DIR}/estimator_lib_test.cpp
    ${UNIT_TESTS_DIR}/test_runner.cpp
    # TODO: add estimator math source files
)
