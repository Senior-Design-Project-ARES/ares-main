# This is my updated cmake file:

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
set(PHY_SOURCES ${THIRD_PARTY_DIR}/STM32CubeH7/Drivers/BSP/Components/lan8742/lan8742.c)
set(LWIP_PORT_SOURCES ${SRC_DIR}/sys_arch.c)


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
    ${HAL_DIR}/Src/stm32h7xx_hal_tim.c
    ${HAL_DIR}/Src/stm32h7xx_hal_uart.c
    ${HAL_DIR}/Src/stm32h7xx_hal_eth_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_pwr_ex.c
    ${HAL_DIR}/Src/stm32h7xx_hal_rcc_ex.c
)

# System and startup files
set(SYSTEM_SOURCES
    ${SRC_DIR}/system_stm32h7xx.c
    ${SRC_DIR}/stm32h7xx_it.c
    ${STARTUP_DIR}/startup_stm32h723xx.s
    ${SRC_DIR}/system_clock.c
)

# IP addressing and Ethernet files
set(LWIP_DIR ${THIRD_PARTY_DIR}/lwip)
set(LWIP_SOURCES
    ${LWIP_DIR}/src/core/init.c
    ${LWIP_DIR}/src/core/mem.c
    ${LWIP_DIR}/src/core/memp.c
    ${LWIP_DIR}/src/core/netif.c
    ${LWIP_DIR}/src/core/pbuf.c
    ${LWIP_DIR}/src/core/timeouts.c
    ${LWIP_DIR}/src/core/udp.c
    ${LWIP_DIR}/src/core/tcp.c
    ${LWIP_DIR}/src/core/tcp_in.c
    ${LWIP_DIR}/src/core/tcp_out.c
    ${LWIP_DIR}/src/core/ipv4/ip4.c
    ${LWIP_DIR}/src/core/ipv4/ip4_addr.c
    ${LWIP_DIR}/src/core/ipv4/etharp.c
    ${LWIP_DIR}/src/core/ipv4/ip4_frag.c
    ${LWIP_DIR}/src/core/ipv4/icmp.c
    ${LWIP_DIR}/src/core/ipv4/igmp.c
    ${LWIP_DIR}/src/core/def.c
    ${LWIP_DIR}/src/core/stats.c
    ${LWIP_DIR}/src/netif/ethernet.c
    ${LWIP_DIR}/src/core/inet_chksum.c
    ${LWIP_DIR}/src/core/ip.c
    
)
set(ETHERNET_SOURCES
    ${SRC_DIR}/ethernetif.c
)

# Ensure C files are compiled as C, not C++
set_source_files_properties(${SRC_DIR}/system_stm32h7xx.c PROPERTIES LANGUAGE C)
set_source_files_properties(${SRC_DIR}/stm32h7xx_it.c PROPERTIES LANGUAGE C)
set_source_files_properties(${HAL_SOURCES} PROPERTIES LANGUAGE C)

# Startup file must be compiled as assembly
set_source_files_properties(${STARTUP_DIR}/startup_stm32h723xx.s PROPERTIES
    LANGUAGE ASM
)

# Create executable target
add_executable(firmware.elf
    main.cpp
    ${HAL_SOURCES}
    ${SYSTEM_SOURCES}
    ${PHY_SOURCES}
    ${LWIP_SOURCES}
    ${ETHERNET_SOURCES}
    ${LWIP_PORT_SOURCES}
)

# Include directories
target_include_directories(firmware.elf PRIVATE
    ${INC_DIR}
    ${SRC_DIR}
    ${CMSIS_DEVICE_DIR}/Include
    ${CMSIS_CORE_DIR}/Include
    ${HAL_DIR}/Inc
    ${HAL_DIR}/Inc/Legacy
    ${THIRD_PARTY_DIR}/STM32CubeH7/Drivers/BSP/Components/lan8742
    ${THIRD_PARTY_DIR}/lwip/src/include
    ${LWIP_DIR}/src/include/arch
)

# Compile definitions
target_compile_definitions(firmware.elf PRIVATE
    STM32H723xx
    USE_HAL_DRIVER
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