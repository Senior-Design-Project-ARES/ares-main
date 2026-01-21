add_executable(firmware.elf)

target_sources(firmware.elf
  PRIVATE
    main.cpp
)

target_link_libraries(firmware.elf
  PRIVATE
    control
)

target_compile_options(firmware.elf PRIVATE
  -mcpu=cortex-m7
  -mthumb
  -ffunction-sections
  -fdata-sections
)

target_link_options(firmware.elf PRIVATE
  -Wl,--gc-sections
)
