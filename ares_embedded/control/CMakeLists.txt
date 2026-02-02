cmake_minimum_required(VERSION 3.20)

# Static control library: cascade controller (IK, wheel PID, Sugeno, limits)
add_library(control STATIC)

target_sources(control
  PRIVATE
    src/limits.cpp
    src/wheel_pid.cpp
    src/inverse_kinematics.cpp
    src/sugeno_rules.cpp
)

target_include_directories(control
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(control PUBLIC cxx_std_20)

target_compile_options(control PRIVATE
  -fno-exceptions
  -fno-rtti
  -ffast-math
)

# Host-only test executable (SITL). Not built when cross-compiling for firmware.
if(NOT CMAKE_CROSSCOMPILING)
  add_executable(test_controller tests/test_controller.cpp)
  target_link_libraries(test_controller PRIVATE control)
  target_include_directories(test_controller PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()
