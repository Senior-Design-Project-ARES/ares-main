# STM32H723ZG 4-Channel Quadrature Encoder Tachometer

Bare-metal firmware for reading four quadrature encoders on the STM32H723ZGT6 (Nucleo-144 board) using hardware timer encoder mode. No HAL dependency — direct register access via CMSIS.

---

## Hardware

- **MCU**: STM32H723ZGT6 on Nucleo-H723ZG
- **Motor**: Pololu 37D 50:1 12V gearmotor, 64 CPR encoder (256 counts/rev in 4x mode)
- **Clock**: 400 MHz, VOS1
- **Debug/Flash**: ST-LINK V3 via OpenOCD

---

## Pin Assignments

| Channel |   Timer   | CH1 Pin | CH2 Pin | Connector                   |
|---------|-----------|---------|---------|-----------------------------|
|   M1    | TIM3, AF2 |   PC6   |   PC7   | CN10 pin 1 (A), pin 11 (B)  |
|   M2    | TIM1, AF1 |   PE9   |   PE11  | CN10 pin 4 (A), pin 6 (B)   |
|   M3    | TIM4, AF2 |   PB6   |   PB7   | CN10 pin 14 (A), pin 16 (B) |
|   M4    | TIM5, AF2 |   PA0   |   PA1   | CN11 pin 28 (A), pin 30 (B) — morpho header |

> **Note on M4 (PA0/PA1)**: These pins must be accessed via the morpho header (CN11 through-holes), not CN10 pin 29. CN10 pin 29 is also PA0 but has solder bridge SB75 and wake-up circuitry that corrupts encoder signals regardless of timer or alternate function used.

---

## Software Architecture

### `encoder_config.h`
Defines the `encoder_hw_config_t` struct and the `g_encoder_hw[4]` configuration table. Each entry specifies the timer, GPIO port/pin for both channels, alternate function, and RCC clock enable mask. The RCC encoding packs the APB bus selector into the upper 2 bits of the clock mask.

### `encoder_backend.h` / `encoder_tim.c`
Hardware timer encoder mode driver. Key functions:
- `encoder_backend_init_all()` — initializes all 4 timers in encoder mode (TI1FP1/TI2FP2, both edges, x4 counting)
- `encoder_backend_poll(uint8_t id)` — no-op for timer backend (counting is fully hardware-driven)
- `encoder_backend_get_and_reset_counts(uint8_t id)` — reads the 16-bit timer counter, computes signed delta from last read with wraparound handling, applies dead-band, resets accumulator

Dead-band filter: deltas in the range (-2, +2) counts are zeroed out to suppress floating input noise at rest.

### `main.c`
Initializes system clock (400 MHz VOS1), UART (USART3, PD8/PD9, 115200 baud), and all encoder channels. Main loop runs at ~20 Hz, reads all 4 channels, converts counts to RPM, and prints:
```
RPM:M1+xxx.xx,M2+xxx.xx,M3+xxx.xx,M4+xxx.xx
```

RPM conversion:
```
RPM = (counts_per_tick * ticks_per_second * 60) / ENCODER_COUNTS_PER_REV
```

---

## Reading Serial Output

Connect via USB and open the serial port at **115200 baud, 8N1**. On Mac/Linux:

```bash
# using screen
screen /dev/tty.usbmodem<XXXX> 115200

# or using minicom
minicom -b 115200 -D /dev/tty.usbmodem<XXXX>
```

On Windows, use PuTTY or any terminal emulator pointed at the correct COM port at 115200 baud.

Output updates at ~20 Hz and looks like:

```
RPM:M1+0.00,M2+0.00,M3+0.00,M4+0.00
RPM:M1+234.38,M2+281.25,M3+257.81,M4+210.94
```

Each field is `<label><sign><value>` where `+` is forward and `-` is reverse.

---

## Getting Raw Encoder Counts

If you need raw counts instead of RPM (e.g. for odometry), call:

```c
int32_t counts = encoder_backend_get_and_reset_counts(uint8_t id);
```

where `id` is 0–3 for M1–M4. This returns the signed count delta since the last call and resets the accumulator. The timer hardware counts all 4 edges per pulse cycle (x4 mode), so one full motor shaft revolution = 256 counts. To convert to output shaft revolutions, divide by the gearbox ratio (50:1):

```
output_shaft_revs = counts / (ENCODER_COUNTS_PER_REV * GEAR_RATIO)
                  = counts / (256 * 50)
                  = counts / 12800
```

---

## Build

This project was developed using PlatformIO and will need to be integrated into your existing CMake or Makefile build system. The critical build flag that must be carried over is:

```
-Wl,-u,_printf_float
```

This enables float support in `printf` and is required for the RPM output to work. Without it, all RPM values will print as `0.00`.

UART is on **USART3, PD8/PD9** at 115200 baud — make sure those pins aren't claimed by another peripheral in your config.

PlatformIO build flags (for reference):
```ini
build_flags = -Wl,-u,_printf_float
```

---

## Flashing

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program .pio/build/nucleo_h723zg/firmware.elf verify reset exit"
```

**Mac USB tip**: If ST-LINK enumerates as `VID:PID 0000:0000`, unplug for 30 seconds, reconnect to a known-good USB-C port, and flash immediately after power-on.

---

## Testing Observations & Expected Jitter

All 4 channels were validated on the bench with a Pololu 37D motor at varying throttle. Observed behaviour:

**At rest (zero throttle)**: All channels read `+0.00` consistently. A dead-band filter suppresses counts in the range (-2, +2) per sample tick to absorb floating input noise — without this, unconnected or lightly loaded inputs showed sporadic ±46–70 RPM spikes.

**Under load**: RPM values are noisy on a per-sample basis due to the 10 ms sample window. At ~300 RPM, expect sample-to-sample variation of roughly ±50–100 RPM. This is normal — the 10 ms window only captures a small number of counts per tick at low speeds, so each count represents ~47 RPM of resolution:

```
RPM resolution per count = 60000 / (256 counts/rev * 10 ms) ≈ 23.4 RPM/count
```

So a 2-count jitter = ~47 RPM of apparent noise. For smooth velocity feedback, apply a moving average or low-pass filter on the consumer side. The raw counts are more suitable for odometry than the RPM output.

**Validated speed range**: Clean readings confirmed from ~50 RPM up to ~750 RPM (unloaded bench test). Expected loaded cruising range per team spec is 100–200 RPM (output shaft: 2–4 RPM, ~0.5–1.0 m/s at the wheel).

---

## Known Board Constraints

- **PA0 on CN10 pin 29**: Unusable for encoder input. SB75 solder bridge connects it to wake-up circuitry that corrupts quadrature signals. Use PA0 via morpho (CN11 pin 28) with TIM5/AF2 instead.
- **PB6/PB7 (M3/TIM4)**: Located on CN10 pins 14/16 (even side). CN10 pins 13/15 are GND — do not use.
- **VOS0 (550 MHz)**: Causes HardFaults on this board. Use VOS1 (400 MHz) only.
- **Encoder mode**: Only works on TIM_CH1 + TIM_CH2. CH3/CH4 cannot be used for quadrature decoding.

---

## Encoder Constants

```c
#define ENCODER_PULSES_PER_REV  64U       // motor encoder CPR
#define ENCODER_COUNTS_PER_REV  256U      // 64 * 4 (quadrature x4 mode)
```
