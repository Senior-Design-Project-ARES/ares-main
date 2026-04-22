# Motor Driver Audit — `user/duriseti/controller`

> **Purpose:** Document Vishnu Duriseti's bare-bones motor driver so CDH can build a proper, shared motor driver interface for both GNC and CDH subsystems.

---

## Branch Overview (sorted by recency)

| Branch | Last Commit | Message |
|--------|-------------|---------|
| `origin/PTCONFIG-path-planning-branch` | 2026-04-03 | updated path tracking code to use config file |
| `origin/GNC-Jetson-Branch` | 2026-04-03 | change environment |
| `origin/gnc-ros2-path-planning-branch` | 2026-04-03 | make turn faster |
| `origin/gnc-target-single-cam-branch` | 2026-04-03 | Single camera testing |
| `origin/Path-tracking-cpp` | 2026-04-01 | Create PP_VLA |
| `origin/GNC-search-algorithm-branch` | 2026-03-31 | Merge branch |
| `origin/gnc-cdh-ethernet` | 2026-03-27 | Updating code for debugging |
| **`origin/user/duriseti/controller`** | **2026-03-27** | **for you riley caus im dumb** ← motor driver here |
| `cdh-wireless` | 2026-03-11 | Add files via upload |
| `origin/cdh_polling_rates_test` | 2026-02-06 | POLLING RATES |
| `gnc-cdh-embedded` | 2026-01-30 | Track ARM toolchain file |
| `origin/user/duriseti/adding-firmware-drivers-for-stm32` | 2026-01-27 | updated readme |

---

## Motor Driver Location

Branch: `origin/user/duriseti/controller`

The motor driver logic is **not in a dedicated file**. It is defined inline in two files:

| File | Role |
|------|------|
| `ares_embedded/firmware/main.cpp` | Motor init + drive loop (forward/reverse 5s each) |
| `ares_embedded/firmware/keyboard_move_motor.cpp` | Teleop demo — same motor structs/functions, arrow-key UART control |

Target hardware: **STM32H723ZG** + **DRV8256P** H-bridge motor IC.

---

## Motor Driver Interface

### Types

```c
// Pin definition for one motor input (IN1 or IN2)
typedef struct {
    GPIO_TypeDef *port;
    uint16_t     pin;
    uint8_t      alternate;  // e.g. GPIO_AF1_TIM1
} motor_pin_t;

// Full config for one motor. IN1 and IN2 may live on different timers.
typedef struct {
    motor_pin_t        in1;
    motor_pin_t        in2;
    TIM_HandleTypeDef *htim_in1;
    TIM_HandleTypeDef *htim_in2;
    uint32_t           channel_in1;
    uint32_t           channel_in2;
} motor_driver_config_t;

// Runtime handle, filled by motor_driver_init()
typedef struct {
    TIM_HandleTypeDef *htim_in1;
    TIM_HandleTypeDef *htim_in2;
    uint32_t           period;   // ARR value (same for both timers if same freq)
    uint32_t           ch_in1;
    uint32_t           ch_in2;
} motor_driver_t;

// DRV8256P truth table
typedef enum {
    MOTOR_BRAKE_LOW  = 0,  // IN1=0,   IN2=0
    MOTOR_FORWARD    = 1,  // IN1=PWM, IN2=0
    MOTOR_REVERSE    = 2,  // IN1=0,   IN2=PWM
    MOTOR_BRAKE_HIGH = 3,  // IN1=100%, IN2=100%
} motor_mode_t;
```

### Functions

```c
// Init a PWM timer. Template enforces 10–40 kHz at compile time.
// Call once per timer, before motor_driver_init().
template <uint32_t PWM_HZ>
void motor_timer_init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance, uint32_t timer_clk_hz);

// Configure GPIO AF pins and start PWM channels for one motor.
void motor_driver_init(const motor_driver_config_t *config, motor_driver_t *ctx);

// Set motor direction and duty cycle (0–100%). Brake modes ignore duty.
void motor_drive(motor_driver_t *ctx, motor_mode_t mode, uint8_t duty_percent);
```

### PWM Timer Setup

Both timers run at **20 kHz** (PSC=0, 64 MHz clock → ARR=3199):

```cpp
static TIM_HandleTypeDef htim1 = {};
static TIM_HandleTypeDef htim2 = {};
motor_timer_init<20000u>(&htim1, TIM1, 64000000u);
motor_timer_init<20000u>(&htim2, TIM2, 64000000u);
```

### 4-Motor Pin Mapping

Both IN1 and IN2 are PWM outputs (not plain GPIO), which is required by the DRV8256P:

| Motor | IN1 Pin | IN1 Timer | IN2 Pin | IN2 Timer |
|-------|---------|-----------|---------|-----------|
| M1 | PE9  (Nucleo D6)  | TIM1_CH1 | PA0  (Nucleo D32) | TIM2_CH1 |
| M2 | PE11 (Nucleo D5)  | TIM1_CH2 | PA1  (Nucleo D33) | TIM2_CH2 |
| M3 | PE13 (Nucleo D3)  | TIM1_CH3 | PB10 (Nucleo D36) | TIM2_CH3 |
| M4 | PE14 (Nucleo D9)  | TIM1_CH4 | PB11 (Nucleo D35) | TIM2_CH4 |

### Teleop Commands (keyboard_move_motor.cpp)

Controls over USART3 at 115200 8N1 (ST-LINK virtual COM):

| Key | Action |
|-----|--------|
| ↑ Arrow | All 4 motors forward |
| ↓ Arrow | All 4 motors reverse |
| ← Arrow | Left motors reverse, right motors forward |
| → Arrow | Left motors forward, right motors reverse |
| Space / S | All motors brake low |

Duty cycle fixed at 22%. Auto-stop after 400 ms with no input.

---

## Encoder Driver Interface

The encoder driver IS in a proper separate file (`encoder_driver.h`) and is well-suited for reuse.

### Public API

```c
// Must call once before any other function. Pass HAL_GetTick().
void encoder_driver_init(uint32_t now_ms);

// Call periodically from main loop. Computes RPM from elapsed time.
void encoder_driver_update(uint32_t now_ms);

// Get wheel speed for encoder_id (0–3).
float   encoder_driver_get_rpm(uint8_t encoder_id);
float   encoder_driver_get_rad_per_sec(uint8_t encoder_id);

// Get raw quadrature counts since last call (clears counter).
int32_t encoder_driver_get_and_reset_counts(uint8_t encoder_id);
```

### Encoder Hardware Mapping

Backend: hardware timer quadrature mode (`encoder_tim.c`).
Resolution: 64 pulses/rev × 4 (quadrature) = **256 counts/rev**.

| Encoder | Timer | Pin A | Pin B |
|---------|-------|-------|-------|
| M1 | TIM3 | PC6 (CN10 pin 1)  | PC7 (CN10 pin 11) |
| M2 | TIM1 | PE9 (CN10 pin 4)  | PE11 (CN10 pin 6) |
| M3 | TIM4 | PB6 (CN10 pin 14) | PB7 (CN10 pin 16) |
| M4 | TIM5 | PA0 (CN11 pin 28) | PA1 (CN11 pin 30) |

> **Note:** TIM1 is shared between M2 encoder and M2 motor PWM (IN1). This is a conflict that will need to be resolved in the proper driver.

### Backend Variants

Three encoder backend implementations exist, selected at compile time via defines:

| File | Mode | When to use |
|------|------|-------------|
| `encoder_tim.c` | Hardware quadrature (timer HW counts) | Production — zero CPU overhead |
| `encoder_isr.c` | GPIO EXTI interrupt | Fallback if timer pins conflict |
| `encoder_polling.c` | Polled GPIO read | Debug/testing only |

---

## Repository File/Folder Structure (`ares_embedded/`)

```
ares_embedded/
├── CMakeLists.txt
├── cmake/
│   └── toolchains/
│       └── arm-none-eabi.cmake
├── control/                        ← GNC control library (C++)
│   ├── include/
│   │   ├── ares_control.hpp        ← top-level include
│   │   ├── config.hpp
│   │   ├── inverse_kinematics.hpp
│   │   ├── limits.hpp
│   │   ├── sugeno_rules.hpp        ← fuzzy logic (Sugeno) controller
│   │   └── wheel_pid.hpp
│   ├── src/
│   │   ├── inverse_kinematics.cpp
│   │   ├── limits.cpp
│   │   ├── sugeno_rules.cpp
│   │   └── wheel_pid.cpp
│   └── tests/
│       └── test_controller.cpp
├── firmware/
│   ├── CMakeLists.txt
│   ├── main.cpp                    ← motor driver code (inline, no header)
│   ├── keyboard_move_motor.cpp     ← teleop demo (duplicates motor driver)
│   ├── test_eth.cpp
│   ├── Drivers/
│   │   ├── encoder/
│   │   │   └── src/
│   │   │       ├── encoder_driver.h    ← clean public API
│   │   │       ├── encoder_driver.c
│   │   │       ├── encoder_backend.h   ← internal backend interface
│   │   │       ├── encoder_config.h    ← HW pin/timer table
│   │   │       ├── encoder_tim.c       ← HW quadrature backend (default)
│   │   │       ├── encoder_isr.c       ← EXTI interrupt backend
│   │   │       └── encoder_polling.c   ← polling backend
│   │   ├── ethernet/
│   │   │   └── src/
│   │   │       ├── ethernet_driver.h/.c
│   │   │       ├── ethernetif.h/.c     ← lwIP netif adapter
│   │   │       ├── lwip_sys_arch.c
│   │   │       ├── lwip_sys_now.c
│   │   │       └── tcp_echo_server.h/.c
│   │   └── uart/
│   │       └── src/
│   │           ├── uart_print.h
│   │           └── uart_print.c        ← USART3 debug println()
│   ├── Inc/
│   │   ├── lwipopts.h
│   │   ├── stm32h7xx_hal_conf.h
│   │   └── stm32h7xx_it.h
│   ├── Src/
│   │   ├── stm32h7xx_it.c
│   │   ├── syscalls.c
│   │   ├── sysmem.c
│   │   └── system_stm32h7xx.c
│   ├── linker/
│   │   └── STM32H723ZGTX_FLASH.ld
│   └── startup/
│       └── startup_stm32h723xx.s
├── docs/
│   ├── BUILDING.md
│   ├── CODE_OF_CONDUCT.md
│   └── CONTRIBUTING.md
├── viz/
│   └── plot_controller_logs.py
├── fw_build.sh
├── run_sitl.sh
└── requirements.txt
```

---

## Issues to Resolve for a Shared CDH/GNC Driver

| # | Issue | Detail |
|---|-------|--------|
| 1 | **No dedicated motor driver file** | `motor_driver_t`, `motor_mode_t`, `motor_driver_init()`, and `motor_drive()` are defined inline in `main.cpp` and copy-pasted into `keyboard_move_motor.cpp`. Need a `motor_driver.h` / `motor_driver.c`. |
| 2 | **TIM1 conflict** | TIM1 is used for both M2 encoder (hardware quadrature) and M2 motor PWM (IN1 on PE9/PE11). These cannot both be active simultaneously — pin mapping needs rework. |
| 3 | **No velocity command interface** | The current API only accepts `(mode, duty_percent)`. GNC needs to send signed velocity or RPM targets; CDH needs to close the loop using encoder feedback. A PID layer on top of `motor_drive()` is needed. |
| 4 | **Hardcoded duty in teleop** | Teleop fixes duty at 22% with no runtime adjustment. The proper driver should accept a normalized `[-1.0, 1.0]` or RPM setpoint. |
| 5 | **C++ template on C driver** | `motor_timer_init<PWM_HZ>()` is a C++ template. If CDH firmware is pure C, this needs to become a runtime frequency parameter or a macro. |

---

## Control Library Deep-Dive

### What Each Component Does

#### `InverseKinematics` — body commands → per-wheel speed commands

```cpp
// Input:  v_cmd [m/s], yaw_rate_cmd [deg/s]
// Output: w_cmd[4] — per-wheel speed setpoints in [deg/s]
void InverseKinematics::compute(float v_cmd, float yaw_rate_cmd_deg, float w_cmd[4]);
```

Differential-drive model. Front wheels (W1, W2) get the full body velocity; rear wheels (W3, W4) add ±yaw correction based on half the rear track width:

```
w1, w2 = v_cmd / r          (front, no yaw contribution)
w3     = (v_cmd - ψ̇ · L/2) / r
w4     = (v_cmd + ψ̇ · L/2) / r
```

Output is `deg/s`, not RPM, not m/s — this is the unit that flows into the PID.

#### `WheelPid` — per-wheel speed error → per-wheel actuator command

```cpp
// Input:  w_cmd[4] [deg/s] from IK,  w_meas[4] [deg/s] from encoders
// Output: u_out[4] — dimensionless actuator commands (see below)
void WheelPid::step(const float w_cmd[4], const float w_meas[4], float u_out[4]);
```

Standard discrete PID with internal integrator per wheel. No saturation inside — caller must clamp with `Limits::clamp_wheel_commands()`.

**Gains (from `config.hpp`, derived from MATLAB):**

| Gain | Value |
|------|-------|
| Kp | 0.0045 |
| Ki | 0.0152 |
| Kd | 0.0005 |
| dt | 0.01 s |

#### `SugenoGains` — optional gain scheduler (currently disabled)

```cpp
// Input:  body-level errors: v_err [m/s], psi_err [deg/s]
// Output: {Kp_mult, Ki_mult, Kd_mult} — multipliers applied to base gains
SugenoGains sugeno_gains(float v_err, float psi_err_deg);
```

`kSugenoEnabled = 0` in `config.hpp` — off by default. When enabled, the caller recomputes PID gains each tick:

```cpp
auto g = sugeno_gains(v_goal - v_meas, psi_goal - psi_meas);
pid.set_gains(kKp * g.Kp_mult, kKi * g.Ki_mult, kKd * g.Kd_mult, dt);
```

---

### What the Control Library Outputs

`WheelPid::step()` writes to `float u_out[4]`.

**What are the units?** The SITL test (`test_controller.cpp`) shows how `u_out` is consumed:

```cpp
// Plant model in test_controller.cpp:
constexpr float kKAct = 220.0f;   // deg/s per actuator unit
constexpr float kTauW = 0.20f;    // wheel first-order lag [s]
// ...
// a = exp(-dt/kTauW), b = kKAct * (1 - a)
w_true[i] = a * w_true[i] + b * u_out[i];   // first-order plant model
```

So in simulation, `u_out` has units of **"actuator units"** where `1.0 = 220 deg/s` of wheel speed in steady state. The sim clamps it to:

```cpp
float u_limit = kWheelSpeedLimitDegPerS / kKAct;  // = 1000 / 220 ≈ ±4.54
Limits::clamp_wheel_commands(u_out, u_out, kNumWheels, -u_limit, u_limit);
```

On real hardware the mapping is: **`u_out[i]` is a signed float roughly in `[-4.54, +4.54]`**. Positive = forward, negative = reverse. `kOutputLimit = 1.0f` in `config.hpp` suggests the intended normalized target is `[-1.0, 1.0]`, but the sim shows this limit is not enforced — the wider ±4.54 is actually used.

**Bottom line: the control library's output is a signed float per wheel. It does not produce duty cycles, RPM, or GPIO states. The caller is responsible for mapping `u_out[i]` to the motor driver.**

---

### Does the Control Library Call `motor_drive()`?

**No. There is zero coupling between `control/` and `firmware/`.** They are completely independent:

- Nothing in `control/include/` or `control/src/` imports anything from `firmware/`
- `firmware/main.cpp` includes `ares_control.hpp` but only uses it in the closed-loop section — and even there, `motor_drive()` is called separately with hardcoded values, not with the PID output
- The SITL test feeds `u_out` directly into a software plant model, never touching `motor_drive()`

**The bridge between control output and motor hardware does not exist yet.** It needs to be written.

---

### The Missing Bridge Layer

This is what needs to be written to connect the two halves. Conceptually:

```c
// For each wheel i, map u_out[i] (signed float) → motor_drive() call
void apply_control_output(motor_driver_t *motors, const float u_out[4]) {
    for (int i = 0; i < 4; i++) {
        float u = u_out[i];
        if (u > DEADBAND) {
            uint8_t duty = (uint8_t)(u / U_MAX * 100.0f);
            motor_drive(&motors[i], MOTOR_FORWARD, duty);
        } else if (u < -DEADBAND) {
            uint8_t duty = (uint8_t)(-u / U_MAX * 100.0f);
            motor_drive(&motors[i], MOTOR_REVERSE, duty);
        } else {
            motor_drive(&motors[i], MOTOR_BRAKE_LOW, 0);
        }
    }
}
```

Where `U_MAX` is the saturation limit (≈4.54 or 1.0 depending on how you normalize) and `DEADBAND` prevents chatter at zero.

---

## MC33926 Driver Replacement

### DRV8256P vs MC33926 Pin Interface

| Signal | DRV8256P | MC33926 |
|--------|----------|---------|
| IN1 | PWM (duty = speed when forward) | GPIO (direction only: HIGH=forward, LOW=reverse) |
| IN2 | PWM (duty = speed when reverse) | GPIO (direction only: complement of IN1) |
| Speed | Implicit in IN1/IN2 PWM duty | Separate `PWM` pin |
| Brake | Both INs = 0 (brake low) or both = 100% | IN1=IN2 or dedicated `D1` pin |

The DRV8256P encodes both direction and speed in two PWM signals. The MC33926 separates them: direction is plain GPIO, speed is a dedicated PWM pin.

### What to Preserve (the control library interface)

The control library knows nothing about the motor driver — it only produces `float u_out[4]`. **You do not need to change anything in `control/` regardless of which motor IC you use.**

The interface you must preserve is the conceptual contract:

```
float u_out[4]  →  [bridge layer]  →  motor_drive() or equivalent
```

Whatever you name the new function, it must accept a signed float per wheel and produce the appropriate IC-specific signals.

### New `motor_driver_config_t` for MC33926

The existing config struct needs a third signal (the dedicated PWM pin):

```c
// New config for MC33926
typedef struct {
    // Direction pins — plain GPIO, not AF
    GPIO_TypeDef *in1_port;
    uint16_t      in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t      in2_pin;
    // Speed — PWM alternate function pin
    GPIO_TypeDef  *pwm_port;
    uint16_t       pwm_pin;
    uint8_t        pwm_alternate;
    TIM_HandleTypeDef *htim_pwm;
    uint32_t           channel_pwm;
} motor_driver_config_t;

// Runtime context
typedef struct {
    GPIO_TypeDef  *in1_port;
    uint16_t       in1_pin;
    GPIO_TypeDef  *in2_port;
    uint16_t       in2_pin;
    TIM_HandleTypeDef *htim_pwm;
    uint32_t           period;
    uint32_t           ch_pwm;
} motor_driver_t;
```

### New `motor_drive()` for MC33926

```c
void motor_drive(motor_driver_t *ctx, motor_mode_t mode, uint8_t duty_percent) {
    if (!ctx) return;
    if (duty_percent > 100u) duty_percent = 100u;

    uint32_t pulse = (uint32_t)duty_percent * (ctx->period + 1u) / 100u;
    if (pulse > ctx->period) pulse = ctx->period;

    switch (mode) {
        case MOTOR_BRAKE_LOW:
            HAL_GPIO_WritePin(ctx->in1_port, ctx->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ctx->in2_port, ctx->in2_pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(ctx->htim_pwm, ctx->ch_pwm, 0);
            break;
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(ctx->in1_port, ctx->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ctx->in2_port, ctx->in2_pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(ctx->htim_pwm, ctx->ch_pwm, pulse);
            break;
        case MOTOR_REVERSE:
            HAL_GPIO_WritePin(ctx->in1_port, ctx->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ctx->in2_port, ctx->in2_pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(ctx->htim_pwm, ctx->ch_pwm, pulse);
            break;
        case MOTOR_BRAKE_HIGH:
            HAL_GPIO_WritePin(ctx->in1_port, ctx->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ctx->in2_port, ctx->in2_pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(ctx->htim_pwm, ctx->ch_pwm, 0);
            break;
    }
}
```

The `motor_mode_t` enum and the bridge layer calling into it remain **identical** — only the internals of `motor_drive()` and the config/init change.
