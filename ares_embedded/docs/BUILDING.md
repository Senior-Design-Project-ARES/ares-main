# Building

The goal is to:
- Develop and validate control logic (PID, wheel speed control, filters, etc.)
- Build and test controller on the host machine
- Develop baseline firmware targets
- Keep the build system minimal and explicit

---

## Prerequisites

### For all builds
- CMake >= 3.20
- A C++20-capable compiler (Clang or GCC recommended)

### For firmware builds
- `arm-none-eabi-gcc` available in your `PATH`

On macOS:
```sh
brew install cmake
```

---

## Build the control library

This builds:
- The `control` static library
- Host-side unit tests

### Configure

```sh
cmake -S . -B build
```

### Build

```sh
cmake --build build
```

---

## Run tests

```sh
ctest --test-dir build
```

Tests are host-only and validate control behavior and correctness.

---

## Firmware Build (STM32H723ZG)

Firmware is cross-compiled for the STM32H723ZG microcontroller using a CMake toolchain file.

### Prerequisites

Install the ARM GCC toolchain:

**On macOS:**
```sh
# IMPORTANT: Install the complete toolchain cask (includes newlib)
# Do NOT install individual packages like arm-none-eabi-gcc
brew install --cask gcc-arm-embedded
```

**Note:** If you already have `arm-none-eabi-gcc` installed separately, uninstall it first:
```sh
brew uninstall arm-none-eabi-gcc arm-none-eabi-binutils arm-none-eabi-gdb
brew install --cask gcc-arm-embedded
```

**On Linux:**
```sh
sudo apt-get install gcc-arm-none-eabi
```

**On Windows:**
Download from [ARM Developer](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)

### Configure

```sh
cmake -S . -B build-firmware \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake
```

### Build

```sh
cmake --build build-firmware
```

The output is `build-firmware/firmware/firmware.elf` - a firmware binary suitable for flashing to the NUCLEO-H723ZG board.

### Flash to Board

If using mac:
```sh
cmake --build build-firmware --target flash
```

This is because of the custom target addition in the firmware `CMakeList.txt`

```sh
add_custom_target(flash
    COMMAND arm-none-eabi-objcopy -O binary firmware.elf firmware.bin
    COMMAND st-flash --connect-under-reset write firmware.bin 0x08000000
    DEPENDS firmware.elf
)
```

Or using STM32CubeProgrammer if on Windows and flash `build-firmware/firmware.elf` using the start address of `0x08000000`.

### Firmware Structure

The firmware includes:
- **CMSIS:** Core peripheral access layer
- **HAL:** Hardware abstraction layer (GPIO, UART, TIM, ETH only)
- **Startup:** Reset and vector table initialization
- **System:** Clock configuration and system initialization
- **main.cpp:** Arduino-like `setup()` and `loop()` entry point

All HAL modules except GPIO, UART, TIM/PWM, and Ethernet are disabled for a minimal footprint.

---

## Directory overview

```
ares_embedded/
├── cmake/toolchains/        # cross-compilation toolchains
├── control/                 # portable control logic (no HAL)
├── firmware/                # embedded firmware integration
├── test/                    # host-side unit tests
└── CMakeLists.txt
```

---

## Design rules (important)

- Control code must be **platform-agnostic**
- Firmware is a **thin integration layer**
- No HAL, hardware, or OS headers in `control/`
- No dynamic allocation unless explicitly justified
- Time (`dt`) and measurements are always passed explicitly
- Determinism is preferred over convenience

---

This setup is intentionally simple to support fast iteration and correctness.
