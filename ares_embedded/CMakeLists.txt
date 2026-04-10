cmake_minimum_required(VERSION 3.20)

project(
    ares_embedded
    VERSION 0.1.0
    DESCRIPTION "Wheel Speed Controller, Firmware, and Unit Tests for ARES Embedded Systems"
    HOMEPAGE_URL "https://github.com/Senior-Design-Project-ARES/ares-main"
    LANGUAGES C CXX ASM
)

add_subdirectory(control)

# Firmware requires the ARM toolchain (e.g. -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake).
# Without it, only the control library is built so you can test controller code with the host compiler.
if(CMAKE_CROSSCOMPILING)
  add_subdirectory(firmware)
else()
  message(STATUS "Skipping firmware (host build). Use -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake to build firmware.")
endif()