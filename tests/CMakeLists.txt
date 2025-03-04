cmake_minimum_required(VERSION 3.22)

# Add coverage flags globally
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")

# Define paths
set(MOCKS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mocks)
set(FFF_DIR ${MOCKS_DIR}/fff)
set(UNIT_TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/unit)
# Full path to the file
set(PATH "/workspaces/cansw_processor_canards/src/drivers/gpio/gpio.c")
get_filename_component(DIR_PATH ${PATH} DIRECTORY)  # /workspaces/cansw_processor_canards/src/drivers/gpio
get_filename_component(DIR_PATH ${DIR_PATH} DIRECTORY)  # /workspaces/cansw_processor_canards/src/drivers
get_filename_component(PROC_SRC_DIR ${DIR_PATH} DIRECTORY)  # /workspaces/cansw_processor_canards/src

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
    target_compile_definitions(${TEST_NAME} PRIVATE
        BOARD_INST_UNIQUE_ID=0x01
        BOARD_TYPE_UNIQUE_ID=0X06
    )
    gtest_discover_tests(${TEST_NAME})
endmacro()

# Make tests automatically generate coverage report after running

add_custom_target(cov ALL
    COMMAND ctest --output-on-failure
    COMMAND lcov --capture --directory ${CMAKE_BINARY_DIR} --output-file src_coverage.info
    COMMAND lcov --extract src_coverage.info ${PROC_SRC_DIR}/drivers/*/* ${PROC_SRC_DIR}/application/*/* --output-file src_coverage_filtered.info
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
    # test files
    ${UNIT_TESTS_DIR}/test_runner.cpp
    ${UNIT_TESTS_DIR}/gpio_test.cpp
    # src files
    ${PROC_SRC_DIR}/drivers/gpio/gpio.c
)

add_test_group(test_uart
    # test files
    ${UNIT_TESTS_DIR}/test_runner.cpp
    ${UNIT_TESTS_DIR}/uart_test.cpp
    # src files
    ${PROC_SRC_DIR}/drivers/uart/uart.c
)

add_test_group(test_estimator_math
    # test files
    ${UNIT_TESTS_DIR}/test_runner.cpp
    ${UNIT_TESTS_DIR}/estimator_lib_test.cpp
    # TODO: add estimator math source files when they exist
)
