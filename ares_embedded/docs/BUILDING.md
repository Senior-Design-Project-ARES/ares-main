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

## Firmware Build (Embedded Target)
Firmware is cross-compiled using a CMake toolchain file.

### Configure
```sh
cmake -S . -B build-firmware \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake
```

### Build
```sh
cmake --build build-firmware
```
The output is an embedded firmware binary (e.g. `firmware.elf`) suitable for flashing.

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
