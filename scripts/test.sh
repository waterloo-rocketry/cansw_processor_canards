#!/bin/bash

# Variables
ROOT_DIR=$(dirname "$(dirname "$(realpath "$0")")") # Root directory of the project
BUILD_DIR="${ROOT_DIR}/tests/build"                # Build directory inside tests
TESTS_DIR="${ROOT_DIR}/tests"                      # Tests directory

# Functions
build_project() {
  echo "Building the project..."
  mkdir -p "$BUILD_DIR"
  cmake -S "$TESTS_DIR" -B "$BUILD_DIR" # Configure project with source and build paths
  cmake --build "$BUILD_DIR" --parallel
  echo "Build completed successfully."
}

run_tests() {
  echo "Running tests..."
  if [ -d "$BUILD_DIR" ]; then
    ctest --test-dir "$BUILD_DIR" --verbose
  else
    echo "Error: Build directory does not exist. Please build the project first."
    exit 1
  fi
}

clean_project() {
  echo "Cleaning the build directory..."
  if [ -d "$BUILD_DIR" ]; then
    rm -rf "$BUILD_DIR"
    echo "Build directory cleaned."
  else
    echo "Build directory does not exist. Nothing to clean."
  fi
}

show_help() {
  echo "Usage: $0 {build|test|clean|help}"
  echo
  echo "Commands:"
  echo "  build    Configure and build the project"
  echo "  test     Run the tests using CTest"
  echo "  clean    Remove the build directory"
  echo "  help     Show this help message"
}

# Main script logic
case "$1" in
  build)
    build_project
    ;;
  test)
    run_tests
    ;;
  clean)
    clean_project
    ;;
  help | *)
    show_help
    ;;
esac
