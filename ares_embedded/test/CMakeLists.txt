add_executable(control_tests)

target_sources(control_tests
  PRIVATE
    source/pid_test.cpp
)

target_link_libraries(control_tests
  PRIVATE
    control
)
