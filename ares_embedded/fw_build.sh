#!/usr/bin/env bash

# Simple cross-platform (POSIX) helper to configure and build the STM32 firmware.
# Works on macOS, Linux, and Windows via WSL or Git Bash, as long as CMake and
# the ARM GCC toolchain (arm-none-eabi-*) are in PATH.
#
# Usage:
#   ./build_firmware.sh            # configure (if needed) and build firmware.elf (Release)
#   ./build_firmware.sh debug      # same, but Debug configuration
#
# The script:
#   - creates/uses a dedicated build directory: ares_embedded/build/firmware-<config>
#   - configures CMake with the arm-none-eabi toolchain
#   - builds the firmware.elf target

set -euo pipefail

# Resolve embedded root (ares_embedded) from this script's location.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EMBEDDED_DIR="${SCRIPT_DIR}"
# Configure from the ares_embedded top-level CMakeLists.txt (adds control + firmware).
SOURCE_DIR="${EMBEDDED_DIR}"
TOOLCHAIN_FILE="${EMBEDDED_DIR}/cmake/toolchains/arm-none-eabi.cmake"

CONFIG="${1:-Release}"
BUILD_DIR="${EMBEDDED_DIR}/build/firmware-${CONFIG}"

echo "=== ARES firmware build ==="
echo "Configuration : ${CONFIG}"
echo "Source        : ${SOURCE_DIR}"
echo "Build dir     : ${BUILD_DIR}"
echo

if [[ ! -f "${TOOLCHAIN_FILE}" ]]; then
  echo "Error: toolchain file not found at:"
  echo "  ${TOOLCHAIN_FILE}"
  echo "Make sure the ares_embedded/cmake/toolchains/arm-none-eabi.cmake file exists."
  exit 1
fi

# Always start from a clean firmware build directory to avoid stale CMake cache.
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

echo "[1/2] Configuring CMake (if needed)..."
cmake -S "${SOURCE_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
  -DCMAKE_BUILD_TYPE="${CONFIG}"

echo "[2/2] Building firmware.elf..."
cmake --build "${BUILD_DIR}" --target firmware.elf -j

echo
echo "Build complete."
echo "Output binary: ${BUILD_DIR}/firmware.elf"

