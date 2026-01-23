cmake_minimum_required(VERSION 3.20)

project(
    ares_embedded
    VERSION 0.1.0
    DESCRIPTION "Wheel Speed Controller, Firmware, and Unit Tests for ARES Embedded Systems"
    HOMEPAGE_URL "https://github.com/Senior-Design-Project-ARES/ares-main"
    LANGUAGES C CXX ASM
)

add_subdirectory(control)
add_subdirectory(firmware)