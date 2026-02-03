# ares_embedded

This is the ares_embedded project - a minimal, clean firmware framework for the STM32H723ZG microcontroller.

## Submodules (required for firmware)

Firmware depends on **STM32CubeH7** as a submodule at `ares_embedded/firmware/third_party/STM32CubeH7`. That repo also has nested submodules (CMSIS device, HAL driver, etc.), so they must be initialized.

**If you are cloning the repo for the first time**, clone with recursion so submodules are fetched:

```sh
# From wherever you clone (e.g. ares-main)
git clone --recurse-submodules <repo-url>
cd ares-main
```

**If you already have the repo but are on this branch for the first time**, or you cloned without submodules, run from the **repository root** (the `ares-main` directory):

```sh
git submodule update --init --recursive
```

This populates `ares_embedded/firmware/third_party/STM32CubeH7` and its nested submodules. Without this step, the firmware build will fail with missing files.

## Hardware

**Target Board:** NUCLEO-H723ZG
- **MCU:** STM32H723ZGT6 (Cortex-M7 @ 520 MHz)
- **Flash:** 1 MB
- **RAM:** 128 KB + 64 KB ITCM
- **Peripherals:** GPIO, UART, TIM/PWM, Ethernet

## Firmware Structure

The firmware follows a clean, minimal structure:

```
firmware/
├── third_party/          # Submodule: run git submodule update --init --recursive
│   └── STM32CubeH7/     # Full STM32CubeH7 repo (Drivers/CMSIS, Drivers/STM32H7xx_HAL_Driver, etc.)
├── Inc/                  # Header files (HAL config)
├── Src/                  # System files
├── startup/              # Startup assembly file
├── linker/               # Linker script
└── main.cpp             # Main application (Arduino-like)
```

## Features

- **Minimal HAL:** Only GPIO, UART, TIM/PWM, and Ethernet enabled
- **No RTOS:** Simple bare-metal execution
- **Arduino-like API:** `setup()` and `loop()` functions
- **Clean build:** No examples, BSP, or middleware
- **Simple workflow:** Almost everything happens in `main.cpp`

## Building and installing

See the [BUILDING](docs/BUILDING.md) document.

## Contributing

See the [CONTRIBUTING](docs/CONTRIBUTING.md) document.

## Code of Conduct

See the [CODE_OF_CONDUCT](docs/CODE_OF_CONDUCT.md) document.

## Running the SITL (Non-Firmware Code)

The SITL runs the controller in simulation on your host machine (no board needed), logs data to CSV, and displays plots.

### One-time setup: Python venv and requirements

From `ares_embedded`:

```sh
python3 -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

You can use `venv` instead of `.venv` if you prefer; the script looks for both.

### Run the pipeline

From `ares_embedded`:

```sh
./run_sitl.sh
```

If you get "permission denied", run `bash run_sitl.sh` instead, or once: `chmod +x run_sitl.sh`.

This will:

1. **Build** the control library and host test.
2. **Run** the controller test (case 1); it writes a CSV to `logs/`.
3. **Plot** the log with matplotlib (three figure windows: body velocity/yaw-rate, wheel speeds, XY path).
4. **Remove** the build directory when done. Log files stay in `logs/`.

To plot an existing log without re-running the test:

```sh
source .venv/bin/activate # to get inside your virtual env (only do this if you aren't already inside your venv)
python3 viz/plot_controller_logs.py logs/<log>.csv
deactivate # to get out of your virtual env
```
