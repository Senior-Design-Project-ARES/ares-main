# ares_embedded

Embedded controls and firmware for ARES on STM32H723ZG, with host-side controller tests and SITL tooling.

Author: Tethracross

## Setup

From the repository root (`ares-main`), initialize submodules:

```sh
git submodule update --init --recursive
```

Then move into the embedded project:

```sh
cd ares_embedded
```

Install/prerequisite instructions:

- Host build dependencies and ARM toolchain: [Build Instructions](docs/BUILDING.md)
- SITL Python environment: [Software-In-The-Loop Instructions](docs/SITL.md)
- Board flashing tools and methods: [Flashing Instructions](firmware/FLASHING.md)

### Host build (control + tests)

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

### Firmware build (cross-compile)

```sh
cmake -S . -B build-firmware -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake
cmake --build build-firmware
```

Flash (if `st-flash` is installed):

```sh
cmake --build build-firmware --target flash
```

## Quick scripts (recommended)

From `ares_embedded`, use the bash helpers (install all code dependencies first - clang/cross compiler, st-link, etc.):

```sh
# Build firmware (and auto-flash if ST target is detected)
./fw_build.sh

# Run SITL pipeline (build host test, run, log, plot)
./run_sitl.sh
```

## Folder intent

- `firmware/Drivers`: board-facing driver modules used by firmware (`encoder`, `motor`, `uart`, `ethernet`) with each module's source under `src/` and optional module docs under `docs/`.
- `firmware/tests`: firmware-side experiment/test entry files (for example keyboard control and UART echo). These are optional app entrypoints you can build instead of the default firmware app.
- `control/tests`: host-side C++ tests for control logic; these run on your development machine via `ctest`.

## Choose which firmware `.cpp` gets flashed

The flashed executable entry file is selected in `firmware/CMakeLists.txt` under `add_executable(firmware.elf ...)`.

Right now it is set to:

- `main_ol.cpp` (active - open loop control through keyboard teleop)

To flash a different app, replace `main_ol.cpp` with one of:

- `main.cpp` for the closed-loop UART command flow
- a file from `firmware/tests` (for example `tests/keyboard.cpp`) when you want test behavior on hardware

After changing it, rebuild with `build-firmware` and re-run the `flash` target.

## Docs

- Build details: [docs/BUILDING.md](docs/BUILDING.md)
- SITL setup: [docs/SITL.md](docs/SITL.md)
- Firmware flashing options: [firmware/FLASHING.md](firmware/FLASHING.md)
