#!/bin/bash

BUILD_DIR="build/cppcheck"
ERROR_LOG="build/cppcheck_errors.log"

# Ensure the build directory exists
mkdir -p "$BUILD_DIR"

rm -f "$ERROR_LOG"

# Check only our project source code in src/, ignoring src/third_party/
cppcheck --enable=all --inconclusive --quiet --error-exitcode=1 \
    --suppress=missingIncludeSystem \
    --suppress=missingInclude \
    --cppcheck-build-dir="$BUILD_DIR" \
    --force \
    --check-level=exhaustive \
    --inline-suppr \
    --std=c11 \
    --addon=misc \
    -isrc/third_party \
    src 2> "$ERROR_LOG"

echo "Errors output to $ERROR_LOG"

# Exit with cppcheck's status code
exit $?
