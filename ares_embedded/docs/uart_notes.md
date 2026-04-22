# UART Notes: ST-LINK VCP RX Recovery

## Context

`cmd.cpp` reads incoming bytes through `uart_try_read_byte()` in polling mode (not IRQ mode).
Telemetry is sent from host over ST-LINK VCP (USART3 path).

Observed behavior before fix:

- RX worked briefly after reset/reflash.
- Then status returned to red timeout and stayed there.
- Recovery required power-cycle or reflash.

## Root Cause (Likely)

The polling receive path relied on:

- `HAL_UART_Receive(..., timeout=0)`

in a tight loop. If UART RX error flags/state latched (for example overrun/framing/noise/parity or HAL state drift), polling could stop yielding bytes and fail to recover consistently in this runtime pattern.

This was **not** caused by an overactive ISR, since this path is not using interrupt-driven RX.

## Fix Implemented

In `firmware/Drivers/uart/src/uart_driver.c`, `uart_try_read_byte()` was hardened:

1. Read UART status directly from `USARTx->ISR`.
2. If RX error flags are set (`ORE`, `FE`, `NE`, `PE`):
   - Clear via `USARTx->ICR` (`ORECF`, `FECF`, `NECF`, `PECF`)
   - Flush stale RX via `USARTx->RQR = RXFRQ`
3. If RX data-ready is set (`RXNE_RXFNE`), read `USARTx->RDR` directly and return byte.
4. If HAL handle state drifts out of ready/error-safe state, perform defensive recovery:
   - `HAL_UART_AbortReceive`
   - `HAL_UART_Abort`
   - `HAL_UART_DeInit`
   - `HAL_UART_Init`

## Result

RX no longer requires power-cycle/reflash between sender runs. The STM can continue seeing new telemetry sessions without getting stuck in a persistent red-timeout state caused by latched UART RX error/state conditions.

## Update 2026-04-15 17: UART RX IRQ Ring Buffer + Binary Debug Telemetry

### Scope

Refactor to make USB->STM command ingest robust for axial velocity + turning rate commands (`V<vx>,Y<yaw>\n`) while keeping the hot path non-blocking and observable from host tooling.

### Firmware Changes

1. `firmware/Drivers/uart/src/uart_driver.c` and `.h`
   - Added interrupt-driven RX with a circular software ring buffer (`UART_RX_RING_SIZE = 512`).
   - Added `uart_driver_irq_handler()` to drain hardware RX in `USART3_IRQHandler`.
   - Added counters for:
     - IRQ RX bytes
     - Overrun/framing/noise/parity errors
     - Ring overflow
   - `uart_try_read_byte()` now reads from the software ring (non-blocking, cheap).
   - Added RX stats APIs:
     - `uart_driver_get_rx_stats(...)`
     - `uart_driver_reset_rx_stats(...)`
   - Kept targeted error handling in IRQ (clear sticky flags; no aggressive flush policy outside true error paths).

2. `firmware/Src/stm32h7xx_it.c`
   - Added:
     - `USART3_IRQHandler()` -> `uart_driver_irq_handler()`
   - This activates the RX IRQ ingestion path for ST-LINK VCP (USART3).

3. `firmware/tests/cmd.cpp`
   - Rewritten for cleaner flow:
     - Drain all available RX bytes each loop.
     - Parse ASCII command lines (`V<vx>,Y<yaw>\n`) without blocking.
     - No `HAL_Delay(...)` in hot loop.
     - No text `print/println` UART output from firmware.
   - Added low-rate binary debug telemetry frame (`DebugFrame`, magic `0xD66D`, 64 bytes + CRC16) containing:
     - command echo (`cmd_vx_m_s`, `cmd_yaw_deg_s`)
     - parser stats (`ok`, parse fail, overflow, rx bytes)
     - UART IRQ/ring/error counters
     - current RX state and sequence number
   - LED behavior updated to solid health-state colors (no blinking):
     - Green = decode OK
     - Yellow = decode fail / malformed line
     - Red = RX timeout (`kUartTimeoutMs = 700`)

### Host Tooling Changes

4. `firmware/tests/uart_over_usb.py`
   - Replaced old status-frame decoder with parser for new `DebugFrame`:
     - `DEBUG_MAGIC = 0xD66D`
     - `DEBUG_FORMAT = "<HIIffBBHIIIIIIIIIIH"`
   - `--read-status` now prints structured telemetry including echoed commands and UART health counters.
   - Important usage note:
     - `--read-debug` assumes newline text and will print gibberish when firmware sends binary frames.
     - For this firmware, use `--read-status` (without `--read-debug`) to inspect telemetry.

### Outcome

- RX command path is now decoupled from polling timing sensitivity.
- Parser and main loop remain non-blocking and lightweight.
- Debug visibility is preserved through binary telemetry instead of ad-hoc prints.
- LED health is always solid (not blinking) while still indicating command link health.

