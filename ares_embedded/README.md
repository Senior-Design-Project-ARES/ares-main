# ares_embedded

This is the ares_embedded project - a minimal, clean firmware framework for the STM32H723ZG microcontroller.

## Hardware

**Target Board:** NUCLEO-H723ZG
- **MCU:** STM32H723ZGT6 (Cortex-M7 @ 520 MHz)
- **Flash:** 1 MB
- **RAM:** 128 KB + 64 KB ITCM
- **Peripherals:** GPIO, UART, TIM/PWM, Ethernet

## Firmware Structure

The firmware follows a clean, minimal structure:

```
firmware/
├── third_party/          # CMSIS and HAL drivers
│   ├── CMSIS/
│   └── STM32H7xx_HAL_Driver/
├── Inc/                  # Header files (HAL config)
├── Src/                  # System files
├── startup/              # Startup assembly file
├── linker/               # Linker script
└── main.cpp             # Main application (Arduino-like)
```

## Features

- **Minimal HAL:** Only GPIO, UART, TIM/PWM, and Ethernet enabled
- **No RTOS:** Simple bare-metal execution
- **Arduino-like API:** `setup()` and `loop()` functions
- **Clean build:** No examples, BSP, or middleware
- **Simple workflow:** Almost everything happens in `main.cpp`

## Building and installing

See the [BUILDING](docs/BUILDING.md) document.

## Contributing

See the [CONTRIBUTING](docs/CONTRIBUTING.md) document.

## Code of Conduct

See the [CODE_OF_CONDUCT](docs/CODE_OF_CONDUCT.md) document.

