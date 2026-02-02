#!/usr/bin/env bash
# Build and run the control library + host test (SITL).
# Edit ARES_EMBEDDED_DIR below if this script is not run from the repo.

# --- Path: change this if the script lives elsewhere ---
ARES_EMBEDDED_DIR=""
# If empty, use the directory containing this script.
if [ -z "$ARES_EMBEDDED_DIR" ]; then
  ARES_EMBEDDED_DIR="$(cd "$(dirname "$0")" && pwd)"
fi

BUILD_DIR="${ARES_EMBEDDED_DIR}/SITL_build"
TEST_EXE="${BUILD_DIR}/control/test_controller"

set -e
cd "$ARES_EMBEDDED_DIR"
echo "Building in: $ARES_EMBEDDED_DIR"
cmake -S . -B $BUILD_DIR
cmake --build $BUILD_DIR

if [ -f "${TEST_EXE}" ]; then
  echo "--- Running controller test ---"
  "${TEST_EXE}"
elif [ -f "${TEST_EXE}.exe" ]; then
  echo "--- Running controller test ---"
  "${TEST_EXE}.exe"
else
  echo "Test executable not found (expected at ${TEST_EXE} or ${TEST_EXE}.exe). Skipping run."
  exit 1
fi

echo "" # empty line
rm -rf $BUILD_DIR
echo "SITL build directory automatically removed... ready to run again."
