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

