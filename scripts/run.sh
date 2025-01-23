#!/bin/bash

# Exit on errors
set -e

# Variables
ROOT_DIR=$(dirname "$(dirname "$(realpath "$0" || echo "$(pwd)/$0")")") # Root directory of the project
BUILD_PRESET="Debug"                              # Default build type (Debug, Release)
TESTS_DIR="${ROOT_DIR}/tests"                     # Tests directory
TESTS_BUILD_DIR="${TESTS_DIR}/build"
BUILD_DIR=""                                      # Build directory, dynamically set

# Functions
set_build_dir() {
  case "$1" in
    gtest)
      BUILD_PRESET="gtest"
      BUILD_DIR="${TESTS_BUILD_DIR}"
      ;;
    Debug)
      BUILD_PRESET="Debug"
      BUILD_DIR="${ROOT_DIR}/build/Debug"
      ;;
    Release)
      BUILD_PRESET="Release"
      BUILD_DIR="${ROOT_DIR}/build/Release"
      ;;
    *)
      echo "Invalid target: $1. Use 'gtest', 'Debug', or 'Release'."
      exit 1
      ;;
  esac
}

build_project() {
  echo "Building the project for target $1 with preset ${BUILD_PRESET}..."
  case "$1" in
    gtest)
      mkdir -p "$BUILD_DIR"
      cmake -S "$TESTS_DIR" -B "$TESTS_BUILD_DIR" -G "Ninja"
      cmake --build "$TESTS_BUILD_DIR" --parallel
      ;;
    Debug|Release)
      mkdir -p "$BUILD_DIR"
      cmake --preset "$BUILD_PRESET"
      cmake --build --preset "$BUILD_PRESET"
      ;;
    *)
      echo "Invalid target: $1. Use 'gtest', 'Debug', or 'Release'."
      exit 1
      ;;
  esac
  echo "Build completed successfully. Output directory: ${BUILD_DIR}"
}

run_tests() {
  echo "Running gtest unit tests..."
  if [ -d "$TESTS_BUILD_DIR" ]; then
    ctest --test-dir "$TESTS_BUILD_DIR" --verbose
    echo "Tests completed successfully."
  else
    echo "Error: tests build directory does not exist. Build the project for gtest first."
    exit 1
  fi
}

clean_project() {
  echo "Cleaning the build directory for target $1..."
  if [ -d "$BUILD_DIR" ]; then
    rm -rf "$BUILD_DIR"
    echo "Build directory cleaned."
  else
    echo "Build directory does not exist. Nothing to clean."
  fi
}

cleanbuild_project() {
  echo "Performing clean and build for target $1..."
  clean_project "$1"
  build_project "$1"
}

run_all() {
  echo "Running full pipeline: Clean -> Build $1 and gtest -> Test"
  clean_project "$1"
  build_project "$1"
  run_tests
}

show_help() {
  echo "Usage: $0 {build|cleanbuild|test|clean|all|help} [target]"
  echo
  echo "Commands:"
  echo "  build [target]         Build the project for [target]"
  echo "  cleanbuild [target]    Clean and build the project for [target]"
  echo "  test                   Build for gtest, and run gtest unit tests"
  echo "  clean [target]         Remove the build directory for [target]"
  echo "  all [target]           Clean, build [target] and gtest, and run gtest unit tests"
  echo "  help                   Show this help message"
  echo
  echo "Targets:"
  echo "  gtest                  Build and run gtest unit tests"
  echo "  Debug                  Build Debug firmware"
  echo "  Release                Build Release firmware"
}

# Main script logic
COMMAND=$1
TARGET=$2

case "$COMMAND" in
  build)
    set_build_dir "$TARGET"
    build_project "$TARGET"
    ;;
  cleanbuild)
    set_build_dir "$TARGET"
    cleanbuild_project "$TARGET"
    ;;
  test)
    set_build_dir "gtest"
    build_project "gtest"
    run_tests
    ;;
  clean)
    set_build_dir "$TARGET"
    clean_project "$TARGET"
    ;;
  all)
    set_build_dir "$TARGET"
    run_all "$TARGET"
    ;;
  help | *)
    show_help
    ;;
esac
