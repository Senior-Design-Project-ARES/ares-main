#!/usr/bin/env bash
# Build and run the control library + host test (SITL), log to logs,
# then run Python viz to plot logs (same layout as actuation_controller_sim.m).

# --- Path: change this if the script lives elsewhere ---
ARES_EMBEDDED_DIR=""
# If empty, use the directory containing this script.
if [ -z "$ARES_EMBEDDED_DIR" ]; then
  ARES_EMBEDDED_DIR="$(cd "$(dirname "$0")" && pwd)"
fi

BUILD_DIR="${ARES_EMBEDDED_DIR}/SITL_build"
TEST_EXE="${BUILD_DIR}/control/test_controller"
LOG_DIR="${ARES_EMBEDDED_DIR}/logs"
VIZ_SCRIPT="${ARES_EMBEDDED_DIR}/viz/plot_controller_logs.py"
VENV_DIR="${ARES_EMBEDDED_DIR}/venv"

set -e
cd "$ARES_EMBEDDED_DIR"
echo "Building in: $ARES_EMBEDDED_DIR"
cmake -S . -B $BUILD_DIR
cmake --build $BUILD_DIR

mkdir -p "$LOG_DIR"

if [ -f "${TEST_EXE}" ]; then
  echo "--- Running controller test (case 1), logging to logs ---"
  "${TEST_EXE}" 1
elif [ -f "${TEST_EXE}.exe" ]; then
  echo "--- Running controller test (case 1), logging to logs ---"
  "${TEST_EXE}.exe" 1
else
  echo "Test executable not found (expected at ${TEST_EXE} or ${TEST_EXE}.exe). Skipping run."
  exit 1
fi

echo ""
rm -rf $BUILD_DIR
echo "Build directory removed. CSVs kept in logs/."

# Pipeline: run Python viz on the log (use venv, .venv, or python3)
CSV_LOG="${LOG_DIR}/controller_sim_case1.csv"
if [ -f "$VIZ_SCRIPT" ] && [ -f "$CSV_LOG" ]; then
  if [ -x "${ARES_EMBEDDED_DIR}/venv/bin/python" ]; then
    PYTHON="${ARES_EMBEDDED_DIR}/venv/bin/python"
  elif [ -x "${ARES_EMBEDDED_DIR}/.venv/bin/python" ]; then
    PYTHON="${ARES_EMBEDDED_DIR}/.venv/bin/python"
  else
    PYTHON="python3"
  fi
  echo "--- Plotting logs (MATLAB sim style, display only) ---"
  "$PYTHON" "$VIZ_SCRIPT" "$CSV_LOG"
else
  [ ! -f "$CSV_LOG" ] && echo "No log at $CSV_LOG; skipping plots."
  [ ! -f "$VIZ_SCRIPT" ] && echo "Viz script not found at $VIZ_SCRIPT; skipping plots."
fi

