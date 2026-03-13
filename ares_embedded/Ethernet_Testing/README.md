This is the Ethernet Testing project base.
***********************************************************************************************************************

The purpose of this base is to set up and test the Ethernet connection on an STM32 NUCLEO-H723ZG.

***********************************************************************************************************************
Hardware/Ethernet Details
***********************************************************************************************************************
**Target Board:** NUCLEO-H723ZG
- **MCU:** STM32H723ZGT6 (Cortex-M7 @ 520 MHz)
- **Flash:** 1 MB
- **RAM:** 128 KB + 64 KB ITCM
- **Peripherals:** GPIO, UART, TIM/PWM, Ethernet

IP address defualt is set at 192.168.0.10
DHCP is enabled

************************************************************************************************************************
Version 0.0.1 is based on ST's example for LwIP-TCP-Echo-Server
 ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************

************************************************************************************************************************
Code layout
************************************************************************************************************************
Main execution:
  * "main.c"
Ethernet execution:
  * "tcp_echo_server.c"
Configuration files:
  * "ethernetif.c"
  * "stm32h7xx_it.c"
  * "syscalls.c"
  * "sysmem.c"
Connection setup:
  * "app_ethernet.c" set up DHCP, LED status indicators
Required middleware
  * "LwIP" 
Required Drivers
  * "STM32H7xx_HAL_Driver"
  * "BSP"
  * "STM32H7xx_HAL_Driver"

***********************************************************************************************************************
Know before you go
***********************************************************************************************************************
## Building and installing

See the [BUILDING](docs/BUILDING.md) document.

## Contributing

See the [CONTRIBUTING](docs/CONTRIBUTING.md) document.

## Code of Conduct

See the [CODE_OF_CONDUCT](docs/CODE_OF_CONDUCT.md) document.
