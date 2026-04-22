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
- Board flashing tools and methods: [Flashing Instructions](docs/FLASHING.md)

### Host build (control + tests)

Use any build folder name (examples below use `build`):

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

### Firmware build (cross-compile)

You can keep using `build`, or replace it with any custom folder name (for example `<build-name>`):

```sh
cmake -S . -B <build-name> -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake
cmake --build <build-name>
```

Flash (if `st-flash` is installed):

```sh
cmake --build <build-name> --target flash
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

After changing it, rebuild with your chosen build directory (for example `build`) and re-run the `flash` target.

## Docs

- Build details: [docs/BUILDING.md](docs/BUILDING.md)
- SITL setup: [docs/SITL.md](docs/SITL.md)
- Firmware flashing options: [docs/FLASHING.md](docs/FLASHING.md)
