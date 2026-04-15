# Firmware CMakeLists.txt for STM32H723ZG

# Define paths (third_party submodule is full STM32CubeH7 repo)
set(FIRMWARE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(THIRD_PARTY_DIR ${FIRMWARE_DIR}/third_party)
set(CMSIS_DIR ${THIRD_PARTY_DIR}/STM32CubeH7/Drivers/CMSIS)
set(HAL_DIR ${THIRD_PARTY_DIR}/STM32CubeH7/Drivers/STM32H7xx_HAL_Driver)
set(INC_DIR ${FIRMWARE_DIR}/Inc)
set(SRC_DIR ${FIRMWARE_DIR}/Src)
set(STARTUP_DIR ${FIRMWARE_DIR}/startup)
set(LINKER_DIR ${FIRMWARE_DIR}/linker)
set(LWIP_DIR ${THIRD_PARTY_DIR}/STM32CubeH7/Middlewares/Third_Party/LwIP)

# CMSIS paths
set(CMSIS_DEVICE_DIR ${CMSIS_DIR}/Device/ST/STM32H7xx)
set(CMSIS_CORE_DIR ${CMSIS_DIR}/Core)

# HAL source files (only enabled modules)
set(HAL_SOURCES
    ${HAL_DIR}/Src/stm32h7xx_hal.c
    ${HAL_DIR}/Src/stm32h7xx_hal_cortex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_dma.c
    ${HAL_DIR}/Src/stm32h7xx_hal_eth.c
    ${HAL_DIR}/Src/stm32h7xx_hal_exti.c
    ${HAL_DIR}/Src/stm32h7xx_hal_flash.c
    ${HAL_DIR}/Src/stm32h7xx_hal_gpio.c
    ${HAL_DIR}/Src/stm32h7xx_hal_pwr.c
    ${HAL_DIR}/Src/stm32h7xx_hal_rcc.c
    ${HAL_DIR}/Src/stm32h7xx_hal_rcc_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_tim.c
    ${HAL_DIR}/Src/stm32h7xx_hal_uart.c
)

# System, startup, and libc glue (syscalls/sysmem)
set(SYSTEM_SOURCES
    ${SRC_DIR}/system_stm32h7xx.c
    ${SRC_DIR}/stm32h7xx_it.c
    ${SRC_DIR}/syscalls.c
    ${SRC_DIR}/sysmem.c
    ${STARTUP_DIR}/startup_stm32h723xx.s
)

# Ensure C files are compiled as C, not C++
set_source_files_properties(${SRC_DIR}/system_stm32h7xx.c PROPERTIES LANGUAGE C)
set_source_files_properties(${SRC_DIR}/stm32h7xx_it.c PROPERTIES LANGUAGE C)
set_source_files_properties(${HAL_SOURCES} PROPERTIES LANGUAGE C)

# Startup file must be compiled as assembly
# set_source_files_properties(${STARTUP_DIR}/startup_stm32h723xx.s PROPERTIES
#     LANGUAGE ASM
# )

# Encoder driver sources (TIM-based quadrature decoding)
set(ENCODER_DIR ${FIRMWARE_DIR}/Drivers/encoder)
set(ENCODER_SOURCES
    ${ENCODER_DIR}/src/encoder_tim.c
    ${ENCODER_DIR}/src/encoder_driver.c
)

# UART driver (configurable USART + printf-style I/O)
set(UART_DIR ${FIRMWARE_DIR}/Drivers/uart)
set(UART_SOURCES
    ${UART_DIR}/src/uart_driver.c
)

# Motor driver sources (MC33926 H-bridge — direction GPIO + PWM speed)
set(MOTOR_DIR ${FIRMWARE_DIR}/Drivers/motor)
set(MOTOR_SOURCES
    ${MOTOR_DIR}/src/motor_driver.c
    ${MOTOR_DIR}/src/motor_bridge.c
)

## Ethernet + LwIP (bare-metal, NO_SYS)
set(ETHERNET_DIR ${FIRMWARE_DIR}/Drivers/ethernet)
set(ETHERNET_SOURCES
    ${ETHERNET_DIR}/src/ethernet_driver.c
    ${ETHERNET_DIR}/src/ethernetif.c
    ${ETHERNET_DIR}/src/tcp_echo_server.c
    ${ETHERNET_DIR}/src/lwip_sys_now.c
    ${ETHERNET_DIR}/src/lwip_sys_arch.c
)

# LwIP sources (minimal set for raw TCP/IPv4 + DHCP)
file(GLOB LWIP_CORE_SOURCES
    ${LWIP_DIR}/src/core/*.c
    ${LWIP_DIR}/src/core/ipv4/*.c
)
set(LWIP_NETIF_SOURCES
    ${LWIP_DIR}/src/netif/ethernet.c
)

# GCC can emit a noisy false-positive on packed DHCP option writes at -O2/-O3.
set_source_files_properties(
    ${LWIP_DIR}/src/core/ipv4/dhcp.c
    PROPERTIES COMPILE_OPTIONS "-Wno-stringop-overflow"
)

# PHY driver (LAN8742)
set(LAN8742_DIR ${THIRD_PARTY_DIR}/STM32CubeH7/Drivers/BSP/Components/lan8742)
set(LAN8742_SOURCES
    ${LAN8742_DIR}/lan8742.c
)

# Create executable target
add_executable(firmware.elf
    # tests/keyboard.cpp
    main.cpp
    ${HAL_SOURCES}
    ${SYSTEM_SOURCES}
    ${ENCODER_SOURCES}
    ${UART_SOURCES}
    ${MOTOR_SOURCES}
    ${ETHERNET_SOURCES}
    ${LWIP_CORE_SOURCES}
    ${LWIP_NETIF_SOURCES}
    ${LAN8742_SOURCES}
)

# Include directories
target_include_directories(firmware.elf PRIVATE
    ${INC_DIR}
    ${SRC_DIR}
    ${ENCODER_DIR}/src
    ${UART_DIR}/src
    ${MOTOR_DIR}/src
    ${ETHERNET_DIR}/src
    ${CMSIS_DEVICE_DIR}/Include
    ${CMSIS_CORE_DIR}/Include
    ${HAL_DIR}/Inc
    ${HAL_DIR}/Inc/Legacy
    ${LWIP_DIR}/src/include
    ${LWIP_DIR}/system
    ${LWIP_DIR}/system/arch
    ${LAN8742_DIR}
)

# Compile definitions
target_compile_definitions(firmware.elf PRIVATE
    STM32H723xx
    USE_HAL_DRIVER
    ENCODER_USE_POLLING=0
    ENCODER_USE_TIM=1
    HSE_VALUE=8000000U
    HSE_STARTUP_TIMEOUT=100U
    HSI_VALUE=64000000U
    LSE_VALUE=32768U
    LSI_VALUE=32000U
    # VDD_VALUE=3300U
)

# Compile options for C++ files
target_compile_options(firmware.elf PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wextra
    -Wno-unused-parameter
)

# C++ specific options
target_compile_options(firmware.elf PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)

# Link options
target_link_options(firmware.elf PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    -T${LINKER_DIR}/STM32H723ZGTX_FLASH.ld
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -nostdlib
    -specs=nano.specs
    -specs=nosys.specs
)

# Link libraries
target_link_libraries(firmware.elf
    PRIVATE
        control
)

# add flash target
add_custom_target(flash
    COMMAND arm-none-eabi-objcopy -O binary firmware.elf firmware.bin
    COMMAND st-flash --connect-under-reset write firmware.bin 0x08000000
    DEPENDS firmware.elf
)
