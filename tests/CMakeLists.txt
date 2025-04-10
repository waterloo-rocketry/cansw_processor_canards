cmake_minimum_required(VERSION 3.22)

# Set coverage data directory
set(COVERAGE_DATA_DIR "${CMAKE_BINARY_DIR}/coverage_data")
file(MAKE_DIRECTORY ${COVERAGE_DATA_DIR})

# Add coverage flags - safely to avoid libgcov errors
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wno-unused-parameter")

# Add linker flags for gcov
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed -Wl,--gc-sections")

# Set coverage flags per-file to avoid conflicts - add atomic update to prevent timestamp errors
set(COVERAGE_COMPILE_FLAGS "--coverage -fprofile-arcs -ftest-coverage -fprofile-correction -fprofile-update=atomic")

# Environment variables to prevent libgcov errors - set globally for all processes
set(ENV{GCOV_PREFIX} "${COVERAGE_DATA_DIR}")
set(ENV{GCOV_PREFIX_STRIP} "4") # Strip first 4 directory levels to get relative paths

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

# Create a global property to track all test targets
set_property(GLOBAL PROPERTY TEST_TARGETS "")

# Define the coverage report output directory explicitly
set(COVERAGE_REPORT_DIR "${CMAKE_BINARY_DIR}/coverage_report")

# Create a helper target that builds all tests first
add_custom_target(build_all_tests ALL
    COMMENT "Building all test executables..."
)
# Make sure Google Test is built before attempting to build any tests
add_dependencies(build_all_tests gtest gtest_main gmock gmock_main)

# Target to run tests with verbose output
# This target will clean previous coverage data, create a fresh directory, and run all tests
add_custom_target(runtests ALL
    # First clear previous coverage data to prevent stale results
    COMMAND ${CMAKE_COMMAND} -E echo "Cleaning previous coverage data..."
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${COVERAGE_DATA_DIR}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${COVERAGE_DATA_DIR}
    
    # Run the tests with verbose output
    COMMAND ${CMAKE_COMMAND} -E echo "Running tests with output..."
    COMMAND ctest --output-on-failure -V
    
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running tests with verbose output..."
)

# Make runtests depend on build_all_tests
add_dependencies(runtests build_all_tests)

# Target to generate coverage report
# This target is part of the default build now
add_custom_target(coverage ALL
    # First ensure the coverage report directory exists
    COMMAND ${CMAKE_COMMAND} -E echo "Creating coverage report directory..."
    COMMAND ${CMAKE_COMMAND} -E make_directory ${COVERAGE_REPORT_DIR}
    
    # Generate coverage data with lcov
    COMMAND ${CMAKE_COMMAND} -E echo "Collecting coverage data..."
    COMMAND lcov -b ${CMAKE_SOURCE_DIR} -d ${CMAKE_BINARY_DIR} -c -o ${CMAKE_BINARY_DIR}/coverage.info
    
    # Filter coverage data to exclude system files, tests, mocks, etc.
    COMMAND ${CMAKE_COMMAND} -E echo "Filtering coverage data..."
    COMMAND lcov --remove ${CMAKE_BINARY_DIR}/coverage.info '/usr/*' '*/external/*' '*/tests/*' '*/mocks/*' -o ${CMAKE_BINARY_DIR}/coverage_filtered.info
    
    # Generate HTML report
    COMMAND ${CMAKE_COMMAND} -E echo "Generating HTML report at ${COVERAGE_REPORT_DIR}..."
    COMMAND genhtml ${CMAKE_BINARY_DIR}/coverage_filtered.info --output-directory=${COVERAGE_REPORT_DIR}
    
    # Print report location
    COMMAND ${CMAKE_COMMAND} -E echo "Coverage report generated at: ${COVERAGE_REPORT_DIR}/index.html"
    
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating coverage report..."
    DEPENDS runtests
)

# helper to create a test executable
# usage: `add_test_group(${test_name} ${src_files})`
# src_files must be a list of the test src file(s) and the proc src file(s) being tested
function(add_test_group TEST_NAME)
    add_executable(${TEST_NAME} ${ARGN})  # Expands all additional arguments as a list
    # all tests get the same common include paths and common libraries
    target_include_directories(${TEST_NAME} PRIVATE ${TEST_COMMON_INCS})
    target_link_libraries(${TEST_NAME} PRIVATE ${TEST_COMMON_LIBS})
    
    # Create a unique coverage directory for this test
    file(MAKE_DIRECTORY "${COVERAGE_DATA_DIR}/${TEST_NAME}")
    
    # Add coverage flags to all source files
    foreach(src_file ${ARGN})
        if(src_file MATCHES ".*\\.c$" AND NOT src_file MATCHES ".*/mocks/.*" AND NOT src_file MATCHES ".*/external/.*")
            set_source_files_properties(${src_file} PROPERTIES 
                COMPILE_FLAGS "${COVERAGE_COMPILE_FLAGS}")
        endif()
    endforeach()
    
    # Link with gcov for coverage
    target_link_libraries(${TEST_NAME} PRIVATE gcov)
    
    # Set coverage environment for this test
    set_target_properties(${TEST_NAME} PROPERTIES 
        ENVIRONMENT "GCOV_PREFIX=${COVERAGE_DATA_DIR}/${TEST_NAME};GCOV_PREFIX_STRIP=4")
    
    target_compile_definitions(${TEST_NAME} PRIVATE
        BOARD_INST_UNIQUE_ID=0x01
        BOARD_TYPE_UNIQUE_ID=0X06
    )
    
    # Register with CTest without running the test during discovery
    # This ensures tests are only discovered once during build configuration
    gtest_discover_tests(${TEST_NAME}
        # Set a reasonable discovery timeout
        TEST_DISCOVERY_TIMEOUT 5
        # Use pre-test discovery mode to prevent running tests twice
        DISCOVERY_MODE PRE_TEST
        # Pass verbosity flags to show test output
        TEST_SUFFIX _detailed
        TEST_PREFIX ""
        PROPERTIES ENVIRONMENT "GTEST_COLOR=1"
        EXTRA_ARGS --gtest_output=xml:${CMAKE_BINARY_DIR}/Test/${TEST_NAME}.xml)
        
    # Add this test to our global property
    get_property(CURRENT_TEST_TARGETS GLOBAL PROPERTY TEST_TARGETS)
    list(APPEND CURRENT_TEST_TARGETS ${TEST_NAME})
    set_property(GLOBAL PROPERTY TEST_TARGETS "${CURRENT_TEST_TARGETS}")
    
    # Add this test as a dependency to build_all_tests
    add_dependencies(build_all_tests ${TEST_NAME})
endfunction()





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

# Add mocks as a static lib, without coverage
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
# Add test groups one by one using the helper function `add_test_group(${test_name} ${src_files})`
# src_files is the list of the test src file(s) and the proc src file(s) being tested
# The list of src_files must be in quotations since it's technically 1 argument
###############################################################################
add_test_group(test_example
    # test file(s)
    ${UNIT_TESTS_DIR}/example_test.cpp
    # src file(s)
    # (example test doesn't actually test any proc source files)
)



add_test_group(test_controller
    # test file(s)
    ${UNIT_TESTS_DIR}/controller_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/controller/controller_algorithm.c
    ${PROC_SRC_DIR}/application/controller/gain_table.c
)