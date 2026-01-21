add_library(control STATIC)

target_sources(control
  PRIVATE
    src/pid.cpp
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
