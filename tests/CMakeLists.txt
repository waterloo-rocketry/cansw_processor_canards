cmake_minimum_required(VERSION 3.14)
project(unit_tests C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define paths
set(MOCKS_DIR ${CMAKE_SOURCE_DIR}/mocks)
set(FFF_DIR ${MOCKS_DIR}/fff)
set(UNIT_TESTS_DIR ${CMAKE_SOURCE_DIR}/unit)

# Add FFF as a header-only library
add_library(fff INTERFACE)
target_include_directories(fff INTERFACE ${FFF_DIR})

# Add mock library
file(GLOB MOCK_SOURCES "${MOCKS_DIR}/*.c")
if(NOT MOCK_SOURCES)
    message(FATAL_ERROR "No source files found in ${MOCKS_DIR}")
endif()
add_library(mocks STATIC ${MOCK_SOURCES})
target_include_directories(mocks PUBLIC ${MOCKS_DIR} ${FFF_DIR})

# Gather test files
file(GLOB TEST_FILES "${UNIT_TESTS_DIR}/*.cpp")
if(NOT TEST_FILES)
    message(FATAL_ERROR "No test files found in ${UNIT_TESTS_DIR}")
endif()

# Create test executable
add_executable(unit_tests ${TEST_FILES})

# Include mocks and FFF
target_include_directories(unit_tests PRIVATE ${MOCKS_DIR} ${FFF_DIR})

# Link libraries
target_link_libraries(unit_tests PRIVATE
    fff           # Fake Function Framework
    mocks         # Mock library
    gtest_main    # GoogleTest main function
)

# Fetch GoogleTest
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Discover and register tests
include(GoogleTest)
gtest_discover_tests(unit_tests)
