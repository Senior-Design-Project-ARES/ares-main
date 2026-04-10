# Ethernet Driver (LwIP TCP Echo) — Usage

This is a **bare-metal (NO_SYS) Ethernet + LwIP** driver for the `ares_embedded/firmware/` build (CMake/Makefile), not a CubeIDE project.

## What’s included

- **Driver sources**: `ares_embedded/firmware/Drivers/ethernet/src/`
  - `ethernet_driver.c/.h`: init + poll entrypoints
  - `ethernetif.c/.h`: STM32H7 HAL ETH + LAN8742 PHY netif glue
  - `tcp_echo_server.c/.h`: minimal raw-API TCP echo server
- **LwIP config**: `ares_embedded/firmware/Inc/lwipopts.h`
- **Linker support**: `ares_embedded/firmware/linker/STM32H723ZGTX_FLASH.ld` contains the ETH DMA descriptor sections

## Network configuration

- **DHCP**: enabled by default (see `LWIP_DHCP` in `lwipopts.h`)
- **Static**: supported via `ethernet_driver_config_t` if you disable DHCP

## How to test

The driver starts a TCP echo server (raw API).

- **Default port**: `7` (configurable)

From a host on the same network, connect to the board’s IP:

```bash
nc <BOARD_IP> 7
```

Anything you type will be echoed back.

## Using it in firmware

In your firmware `setup()` (or early init), call `ethernet_driver_init(...)`, then call `ethernet_driver_poll()` frequently in your main loop.

Example:

```c
#include "ethernet_driver.h"

void setup(void)
{
  ethernet_driver_config_t cfg = {0};
  cfg.use_dhcp = 1;
  cfg.tcp_port = 7;
  ethernet_driver_init(&cfg);
}

void loop(void)
{
  ethernet_driver_poll();
}
```

## File map (high-signal)

- **Init + poll entrypoints**: `Drivers/ethernet/src/ethernet_driver.c`
- **ETH netif glue + PHY + DMA sections**: `Drivers/ethernet/src/ethernetif.c`
- **Echo server**: `Drivers/ethernet/src/tcp_echo_server.c`

## Notes / gotchas

- **ETH DMA memory**: DMA descriptors / RX pool must live in D2 SRAM (`0x30000000...`). The firmware linker script must keep the `.lwip_sec` region.
- **MPU/cache**: the driver configures an MPU region for `0x30000000` to avoid DCache coherency issues. If you later move buffers or change size, update the MPU region too.
- **Pin mapping**: `ethernetif.c` sets up RMII pins for NUCLEO-H723ZG. If you change boards, update `HAL_ETH_MspInit`.

