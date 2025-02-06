cmake_minimum_required(VERSION 3.22)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_definitions(GTEST)

# Define paths
set(MOCKS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mocks)
set(FFF_DIR ${MOCKS_DIR}/fff)
set(UNIT_TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/unit)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)  

# Add FFF as a header-only library
add_library(fff INTERFACE)
target_include_directories(fff INTERFACE ${FFF_DIR})

# Add mock library
file(GLOB MOCK_SOURCES "${MOCKS_DIR}/*.c")
if(NOT MOCK_SOURCES)
    message(FATAL_ERROR "No source files found in ${MOCKS_DIR}")
endif()
add_library(mocks STATIC ${MOCK_SOURCES})
target_include_directories(mocks PUBLIC ${MOCKS_DIR} ${FFF_DIR} ${SRC_DIR})

# Add source files from specific subdirectories
file(GLOB_RECURSE APPLICATION_FILES "${SRC_DIR}/application/*.c")
file(GLOB_RECURSE DRIVERS_FILES "${SRC_DIR}/drivers/*.c")

# Combine included source files
set(SRC_FILES ${APPLICATION_FILES} ${DRIVERS_FILES})
if(NOT SRC_FILES)
    message(FATAL_ERROR "No source files found in application or drivers directories")
endif()
add_library(proc_src STATIC ${SRC_FILES})

# Include directories
target_include_directories(proc_src PUBLIC
    # Link mocks instead of stm32 hal
    ${MOCKS_DIR}
    ${MOCKS_DIR}/fff
    ${SRC_DIR}/third_party
    # include canlib headers but we will mock/stub the implementations
    ${SRC_DIR}/third_party/canlib
    ${SRC_DIR}
)

# Gather test files
file(GLOB TEST_FILES "${UNIT_TESTS_DIR}/*.cpp")
if(NOT TEST_FILES)
    message(FATAL_ERROR "No test files found in ${UNIT_TESTS_DIR}")
endif()

# Create test executable
add_executable(tests ${TEST_FILES})

# Include directories
target_include_directories(tests PRIVATE
    ${MOCKS_DIR}
    ${FFF_DIR}
    ${SRC_DIR}
)

# copied from the main project cmakelists
target_compile_definitions(proc_src PUBLIC
    BOARD_TYPE_UNIQUE_ID=0x01
    BOARD_INST_UNIQUE_ID=0X06
)

# Link libraries
target_link_libraries(tests PRIVATE
    fff           # Fake Function Framework
    mocks         # Mock library
    proc_src      # Source code library
    gtest_main    # GoogleTest main function
)

# Add GoogleTest as a subdirectory
add_subdirectory(external/googletest)

# Discover and register tests
include(GoogleTest)

gtest_discover_tests(tests)

