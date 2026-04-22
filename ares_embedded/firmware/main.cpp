/**
 * @file main.cpp
 * @brief Closed-loop wheel PID:
 * @author: Vishnu Duriseti
 *
 * Uses ares_control.hpp (IK + WheelPid + Limits). IK wheel order matches
 * motors[] (LR, LF, RR, RF). Control runs every control::kDt; encoder is
 * sampled at 2 Hz (held z.o.h. between samples for PID feedback).
 */

 #include <algorithm>
 #include <cmath>
 #include <cstdint>
 #include <cstdlib>
 
 #include "stm32h7xx.h"
 #include "stm32h7xx_hal.h"
 
 #include "ares_control.hpp"
 #include "encoder_config.h"
 #include "encoder_driver.h"
 #include "motor_config.h"
 #include "motor_driver.h"
 
 #include "uart_driver.h"
 
 extern "C" {
 void SystemInit(void);
 void _init(void);
 }
 
 namespace {
 
 /** u in [-1, 1] → duty 0–100 %, sign → direction. */
 void apply_actuator_to_motors(const float u_out[control::kNumWheels],
                               motor_driver_t *motors)
 {
     for (int i = 0; i < control::kNumWheels; ++i) {
         const float   u    = u_out[i];
         const float   mag  = std::fabs(u) * 100.f;
         const uint8_t duty = static_cast<uint8_t>(mag);
 
         // print("mag: %f | duty: %d ", mag, static_cast<int>(duty));
 
         if (duty == 0u) {
             motor_drive(&motors[i], MOTOR_BRAKE, 0u);
         } else if (u > 0.f) {
             motor_drive(&motors[i], MOTOR_FORWARD, duty);
         } else {
             motor_drive(&motors[i], MOTOR_REVERSE, duty);
         }
     }
 }

/** Brake all wheels (no UART / red link — do not run closed-loop). */
void stop_all_motors(motor_driver_t *motors)
{
    if (motors == nullptr) {
        return;
    }
    for (int i = 0; i < control::kNumWheels; ++i) {
        motor_drive(&motors[i], MOTOR_BRAKE, 0u);
    }
 }
 
 /** Encoder sample period [ms] (2 Hz). */
 constexpr uint32_t kEncoderSamplePeriodMs = 10U;
constexpr uint8_t  kLineBufferSize        = 64U;
constexpr uint32_t kUartTimeoutMs         = 700U;

enum class RxState : uint8_t
{
    kNoRx = 0U,
    kDecodeOk,
    kDecodeFail
};

enum class LedHealth : uint8_t
{
    kRed = 0U,
    kYellow,
    kGreen
};
 
 /** Output-shaft deg/s from encoder delta over dt_sample_s [s] since last read. */
 void wheel_meas_deg_per_s_hw(float w_meas_hw[control::kNumWheels], float dt_sample_s)
 {
     const float cpr_motor = static_cast<float>(ENCODER_COUNTS_PER_REV);
     const float gear      = static_cast<float>(ENCODER_GEAR_RATIO);
     const float dt        = std::max(dt_sample_s, control::kDt);
 
     const int32_t c_lr = encoder_driver_get_and_reset_counts(WHEEL_IDX_LR);
     const int32_t c_lf = encoder_driver_get_and_reset_counts(WHEEL_IDX_LF);
     const int32_t c_rr = -encoder_driver_get_and_reset_counts(WHEEL_IDX_RR);
     const int32_t c_rf = -encoder_driver_get_and_reset_counts(WHEEL_IDX_RF); // TODO: change to WHEEL_IDX_RF or fix encoder shit
 
     const int32_t counts[4] = {c_lr, c_lf, c_rr, c_rf};
 
     for (int i = 0; i < control::kNumWheels; ++i) {
         const float motor_rev = static_cast<float>(counts[i]) / cpr_motor;
         const float out_rev   = motor_rev / gear;
         w_meas_hw[i]          = (out_rev * 360.f) / dt;
     }
 }
 
struct ControlRuntime
 {
    float    w_meas_hw[control::kNumWheels]{};
    uint32_t last_enc_ms{0U};
};
 
bool parse_ascii_cmd_line(const char *line, float *vx, float *yaw_rate)
{
    if (line == nullptr || vx == nullptr || yaw_rate == nullptr) {
        return false;
    }
    if (line[0] != 'V' && line[0] != 'v') {
        return false;
    }

    char *end = nullptr;
    const float parsed_vx = std::strtof(line + 1, &end);
    if (end == (line + 1) || *end != ',') {
        return false;
    }
    ++end;
    if (*end != 'Y' && *end != 'y') {
        return false;
    }
    ++end;
    const float parsed_yaw = std::strtof(end, &end);
    if (end == nullptr || *end != '\0') {
        return false;
    }

    *vx = parsed_vx;
    *yaw_rate = parsed_yaw;
    return true;
}

void run(float axial_vel_m_s, float turning_rate_deg_s,
         control::InverseKinematics *ik, control::WheelPid *pid,
         motor_driver_t *motors, ControlRuntime *rt)
{
    if (ik == nullptr || pid == nullptr || motors == nullptr || rt == nullptr) {
        return;
    }
 
    float    w_cmd[control::kNumWheels]{};
    float    u_hw[control::kNumWheels]{};
    const uint32_t now = HAL_GetTick();
    const bool take_enc =
        (rt->last_enc_ms == 0U) || ((now - rt->last_enc_ms) >= kEncoderSamplePeriodMs);
    if (take_enc) {
        const float dt_enc_s =
            (rt->last_enc_ms == 0U)
                ? std::max(static_cast<float>(now) * 0.001f, control::kDt)
                : std::max(static_cast<float>(now - rt->last_enc_ms) * 0.001f, control::kDt);
        wheel_meas_deg_per_s_hw(rt->w_meas_hw, dt_enc_s);
        rt->last_enc_ms     = now;
    }
 
    ik->compute(axial_vel_m_s, turning_rate_deg_s, w_cmd);
    control::Limits::clamp_wheel_commands(w_cmd, w_cmd, control::kNumWheels,
                                          -control::kWheelSpeedLimitDegPerS,
                                          control::kWheelSpeedLimitDegPerS);
    pid->step(w_cmd, rt->w_meas_hw, u_hw);
    control::Limits::clamp_wheel_commands(
        u_hw, u_hw, control::kNumWheels, -control::kOutputLimit,
        control::kOutputLimit);

    u_hw[3] = u_hw[2]; // motor mirror hack
    apply_actuator_to_motors(u_hw, motors);
 }
 
RxState poll_uart_ascii_cmd(float *axial_vel_m_s, float *turning_rate_deg_s)
{
    if (axial_vel_m_s == nullptr || turning_rate_deg_s == nullptr) {
        return RxState::kNoRx;
    }

    static char    line_buf[kLineBufferSize]{};
    static uint8_t idx = 0U;
    uint8_t byte = 0U;
    bool got_rx = false;
    bool decode_ok = false;
    bool decode_fail = false;

    while (uart_try_read_byte(&byte)) {
        got_rx = true;
        if (byte == '\r') {
            continue;
        }
        if (byte != '\n') {
            if (idx < static_cast<uint8_t>(kLineBufferSize - 1U)) {
                line_buf[idx++] = static_cast<char>(byte);
            } else {
                idx = 0U;
                decode_fail = true;
            }
            continue;
        }

        line_buf[idx] = '\0';
        if (idx > 0U && parse_ascii_cmd_line(line_buf, axial_vel_m_s, turning_rate_deg_s)) {
            decode_ok = true;
        } else if (idx > 0U) {
            decode_fail = true;
        }
        idx = 0U;
    }

    if (decode_ok) {
        return RxState::kDecodeOk;
    }
    if (decode_fail) {
        return RxState::kDecodeFail;
    }
    return got_rx ? RxState::kDecodeFail : RxState::kNoRx;
}

void leds_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOE, &gpio);
}

void set_led_health(LedHealth health)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (health == LedHealth::kGreen) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, (health == LedHealth::kYellow) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (health == LedHealth::kRed) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

 } // namespace
 
 int main(void)
 {
     HAL_Init();
     uart_driver_init_default();
     encoder_driver_init(HAL_GetTick());
    leds_init();
 
     static TIM_HandleTypeDef htim_pwm = {};
     motor_timer_init(&htim_pwm, MOTOR_PWM_TIMER, MOTOR_PWM_HZ, MOTOR_TIMER_CLK_HZ);
 
     static motor_driver_config_t configs[MOTOR_COUNT] = MOTOR_DRIVER_CONFIGS(&htim_pwm);
     static motor_driver_t        motors[MOTOR_COUNT]  = {};
 
     for (int i = 0; i < MOTOR_COUNT; ++i) {
         motor_driver_init(&configs[i], &motors[i]);
     }
 
     control::InverseKinematics ik;
     control::WheelPid          pid(control::kKp, control::kKi, control::kKd, control::kKf, control::kDt);
     pid.reset();
 
   HAL_Delay(5000U); /* safety delay */

    ControlRuntime rt{};
    float          axial_vel_cmd_m_s   = 0.f;
    float          turning_rate_cmd_deg_s = 0.f;
   uint32_t       last_uart_rx_ms = HAL_GetTick();
   LedHealth      led_health = LedHealth::kRed;
   set_led_health(led_health);

    while (true) {
        const uint32_t now_ms = HAL_GetTick();
        const RxState status = poll_uart_ascii_cmd(&axial_vel_cmd_m_s, &turning_rate_cmd_deg_s);

        if (status == RxState::kDecodeOk) {
            led_health = LedHealth::kGreen;
            last_uart_rx_ms = now_ms;
        } else if (status == RxState::kDecodeFail) {
            led_health = LedHealth::kYellow;
            last_uart_rx_ms = now_ms;
        } else if ((now_ms - last_uart_rx_ms) > kUartTimeoutMs) {
            led_health = LedHealth::kRed;
        }
        set_led_health(led_health);

        static LedHealth prev_led = LedHealth::kGreen;
        if (led_health == LedHealth::kRed) {
            stop_all_motors(motors);
            if (prev_led != LedHealth::kRed) {
                pid.reset();
            }
        } else {
            run(axial_vel_cmd_m_s, turning_rate_cmd_deg_s, &ik, &pid, motors, &rt);
        }
        prev_led = led_health;
        HAL_Delay(static_cast<uint32_t>(control::kDt * 1000.f));
     }
 }
 
 extern "C" {
 void _init(void) {}
 }
 