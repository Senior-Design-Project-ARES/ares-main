/**
 * @file main_ol.cpp
 * @brief Open-loop keyboard teleop using IK + encoder reporting over UART.
 *
 * Merges:
 *  - firmware/main.cpp: IK wheel command path and encoder speed conversion
 *  - firmware/tests/keyboard.cpp: WASD/arrow serial teleop UX
 *
 * Controls over USART3 (115200 8N1):
 *   W/S forward/reverse (clears turn — pure axial until A/D).
 *   A/D yaw only while W or S is active (forward+turn / reverse+turn).
 *   Space or X stop.
 *   I/K increase/decrease linear speed setpoint magnitude.
 *   O/L increase/decrease turning-rate setpoint magnitude.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>

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

static_assert(MOTOR_COUNT == ENCODER_COUNT,
              "motors[] and encoder_driver must use the same wheel index (0..3)");

enum class LedHealth : uint8_t {
    kRed = 0U,
    kYellow,
    kGreen
};

struct TeleopState {
    int8_t axial_dir{0};  // -1 reverse, 0 stop, +1 forward
    int8_t turn_dir{0};   // -1 right, 0 none, +1 left
    float linear_speed_m_s{0.40f};
    float turn_rate_deg_s{35.0f};
};

struct Runtime {
    float    w_cmd[control::kNumWheels]{};
    float    w_meas[control::kNumWheels]{};
    uint32_t last_enc_ms{0U};
    uint32_t last_cmd_ms{0U};
    uint32_t last_print_ms{0U};
};

constexpr uint32_t kEncoderSamplePeriodMs = 20U;
constexpr uint32_t kPrintPeriodMs         = 500U;
constexpr uint32_t kCmdTimeoutMs          = 60000U;
constexpr float    kLinearStepMps         = 0.05f;
constexpr float    kLinearMinMps          = 0.05f;
constexpr float    kLinearMaxMps          = 2.00f;
constexpr float    kTurnStepDegps         = 5.0f;
constexpr float    kTurnMinDegps          = 5.0f;
constexpr float    kTurnMaxDegps          = 180.0f;

constexpr float clampf(float value, float lo, float hi)
{
    return (value < lo) ? lo : ((value > hi) ? hi : value);
}

const char *teleop_cmd_name(const TeleopState &teleop)
{
    const int8_t td = (teleop.axial_dir != 0) ? teleop.turn_dir : 0;
    if (teleop.axial_dir > 0 && td > 0) {
        return "FORWARD+LEFT";
    }
    if (teleop.axial_dir > 0 && td < 0) {
        return "FORWARD+RIGHT";
    }
    if (teleop.axial_dir < 0 && td > 0) {
        return "REVERSE+LEFT";
    }
    if (teleop.axial_dir < 0 && td < 0) {
        return "REVERSE+RIGHT";
    }
    if (teleop.axial_dir > 0) {
        return "FORWARD";
    }
    if (teleop.axial_dir < 0) {
        return "REVERSE";
    }
    return "STOP";
}

void stop_all_motors(motor_driver_t *motors)
{
    if (motors == nullptr) {
        return;
    }
    for (int i = 0; i < control::kNumWheels; ++i) {
        motor_drive(&motors[i], MOTOR_BRAKE, 0u);
    }
}

void apply_open_loop_to_motors(const float w_cmd[control::kNumWheels],
                               motor_driver_t *motors)
{
    if (motors == nullptr) {
        return;
    }

    for (int i = 0; i < control::kNumWheels; ++i) {
        const float w =
            clampf(w_cmd[i], -control::kWheelSpeedLimitDegPerS, control::kWheelSpeedLimitDegPerS);
        const float normalized = w / control::kWheelSpeedLimitDegPerS;  // [-1,1]
        const float duty_f = std::fabs(normalized) * 100.0f;
        const uint8_t duty = static_cast<uint8_t>(clampf(duty_f, 0.0f, 100.0f));

        if (duty == 0U) {
            motor_drive(&motors[i], MOTOR_BRAKE, 0U);
        } else if (normalized > 0.0f) {
            motor_drive(&motors[i], MOTOR_FORWARD, duty);
        } else {
            motor_drive(&motors[i], MOTOR_REVERSE, duty);
        }
    }
}

void wheel_meas_deg_per_s(float w_meas[control::kNumWheels], float dt_sample_s)
{
    const float cpr_motor = static_cast<float>(ENCODER_COUNTS_PER_REV);
    const float gear      = static_cast<float>(ENCODER_GEAR_RATIO);
    const float dt        = std::max(dt_sample_s, control::kDt);

    const int32_t c_lr = encoder_driver_get_and_reset_counts(WHEEL_IDX_LR);
    const int32_t c_lf = encoder_driver_get_and_reset_counts(WHEEL_IDX_LF);
    const int32_t c_rr = -encoder_driver_get_and_reset_counts(WHEEL_IDX_RR);
    const int32_t c_rf = -encoder_driver_get_and_reset_counts(WHEEL_IDX_RF);
    const int32_t counts[control::kNumWheels] = {c_lr, c_lf, c_rr, c_rf};

    for (int i = 0; i < control::kNumWheels; ++i) {
        const float motor_rev = static_cast<float>(counts[i]) / cpr_motor;
        const float out_rev   = motor_rev / gear;
        w_meas[i]             = (out_rev * 360.0f) / dt;
    }
}

void update_command_from_mode(const TeleopState &teleop, float *vx_mps, float *yaw_degps)
{
    if (vx_mps == nullptr || yaw_degps == nullptr) {
        return;
    }

    *vx_mps     = static_cast<float>(teleop.axial_dir) * teleop.linear_speed_m_s;
    *yaw_degps = (teleop.axial_dir != 0)
                     ? static_cast<float>(teleop.turn_dir) * teleop.turn_rate_deg_s
                     : 0.0f;
}

bool poll_teleop_input(TeleopState *teleop, bool *had_decode_error)
{
    if (teleop == nullptr || had_decode_error == nullptr) {
        return false;
    }

    *had_decode_error = false;
    bool got_update = false;
    static uint8_t esc_state = 0U;  // 0 normal, 1 after ESC, 2 ESC[, 3 ESCO
    uint8_t c = 0U;

    while (uart_try_read_byte(&c)) {
        if (esc_state == 0U) {
            if (c == 0x1BU) {
                esc_state = 1U;
                continue;
            }

            switch (c) {
                case 'w':
                case 'W':
                    teleop->axial_dir = 1;
                    teleop->turn_dir  = 0;
                    got_update        = true;
                    break;
                case 's':
                case 'S':
                    teleop->axial_dir = -1;
                    teleop->turn_dir  = 0;
                    got_update        = true;
                    break;
                case 'a':
                case 'A':
                    if (teleop->axial_dir != 0) {
                        teleop->turn_dir = 1;
                    } else {
                        teleop->turn_dir = 0;
                    }
                    got_update = true;
                    break;
                case 'd':
                case 'D':
                    if (teleop->axial_dir != 0) {
                        teleop->turn_dir = -1;
                    } else {
                        teleop->turn_dir = 0;
                    }
                    got_update = true;
                    break;
                case 'x':
                case 'X':
                case ' ':
                    teleop->axial_dir = 0;
                    teleop->turn_dir = 0;
                    got_update = true;
                    break;
                case 'i':
                case 'I':
                    teleop->linear_speed_m_s = clampf(
                        teleop->linear_speed_m_s + kLinearStepMps, kLinearMinMps, kLinearMaxMps);
                    got_update = true;
                    break;
                case 'k':
                case 'K':
                    teleop->linear_speed_m_s = clampf(
                        teleop->linear_speed_m_s - kLinearStepMps, kLinearMinMps, kLinearMaxMps);
                    got_update = true;
                    break;
                case 'o':
                case 'O':
                    teleop->turn_rate_deg_s = clampf(
                        teleop->turn_rate_deg_s + kTurnStepDegps, kTurnMinDegps, kTurnMaxDegps);
                    got_update = true;
                    break;
                case 'l':
                case 'L':
                    teleop->turn_rate_deg_s = clampf(
                        teleop->turn_rate_deg_s - kTurnStepDegps, kTurnMinDegps, kTurnMaxDegps);
                    got_update = true;
                    break;
                case '\r':
                case '\n': break;
                default: *had_decode_error = true; break;
            }
            continue;
        }

        if (esc_state == 1U) {
            if (c == '[') {
                esc_state = 2U;
            } else if (c == 'O') {
                esc_state = 3U;
            } else {
                esc_state = 0U;
                *had_decode_error = true;
            }
            continue;
        }

        switch (c) {
            case 'A':
                teleop->axial_dir = 1;
                teleop->turn_dir  = 0;
                got_update        = true;
                break;
            case 'B':
                teleop->axial_dir = -1;
                teleop->turn_dir  = 0;
                got_update        = true;
                break;
            case 'C':
                if (teleop->axial_dir != 0) {
                    teleop->turn_dir = -1;
                } else {
                    teleop->turn_dir = 0;
                }
                got_update = true;
                break;
            case 'D':
                if (teleop->axial_dir != 0) {
                    teleop->turn_dir = 1;
                } else {
                    teleop->turn_dir = 0;
                }
                got_update = true;
                break;
            default: *had_decode_error = true; break;
        }
        esc_state = 0U;
    }

    return got_update;
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

void print_status(uint32_t t_ms, const TeleopState &teleop, const Runtime &rt, float vx_mps,
                  float yaw_degps)
{
    println(
        "[%lu ms] CMD=%s vx=%.2f[m/s] yaw=%.1f[deg/s] tuning(v=%.2f, yaw=%.1f)",
        static_cast<unsigned long>(t_ms),
        teleop_cmd_name(teleop),
        vx_mps,
        yaw_degps,
        teleop.linear_speed_m_s,
        teleop.turn_rate_deg_s);
    println(
        "[%lu ms] W_CMD deg/s LR=%.1f LF=%.1f RR=%.1f RF=%.1f",
        static_cast<unsigned long>(t_ms),
        rt.w_cmd[WHEEL_IDX_LR],
        rt.w_cmd[WHEEL_IDX_LF],
        rt.w_cmd[WHEEL_IDX_RR],
        rt.w_cmd[WHEEL_IDX_RF]);
    println(
        "[%lu ms] W_ENC deg/s LR=%.1f LF=%.1f RR=%.1f RF=%.1f",
        static_cast<unsigned long>(t_ms),
        rt.w_meas[WHEEL_IDX_LR],
        rt.w_meas[WHEEL_IDX_LF],
        rt.w_meas[WHEEL_IDX_RR],
        rt.w_meas[WHEEL_IDX_RF]);
}

}  // namespace

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
    TeleopState teleop{};
    Runtime rt{};
    rt.last_cmd_ms = HAL_GetTick();
    rt.last_print_ms = rt.last_cmd_ms;
    LedHealth led_health = LedHealth::kRed;
    set_led_health(led_health);

    HAL_Delay(5000U);  // safety delay
    const uint32_t boot_ms = HAL_GetTick();
    println("[%lu ms] Open-loop keyboard teleop ready (USART3 @115200).",
          static_cast<unsigned long>(boot_ms));
    println("[%lu ms] W/S=axial (clears turn). A/D=yaw only while moving. Space/X stop. I/K O/L tune.",
          static_cast<unsigned long>(boot_ms));

    while (true) {
        const uint32_t now_ms = HAL_GetTick();
        bool had_decode_error = false;
        const bool got_cmd = poll_teleop_input(&teleop, &had_decode_error);
        if (got_cmd) {
            rt.last_cmd_ms = now_ms;
            led_health = had_decode_error ? LedHealth::kYellow : LedHealth::kGreen;
        } else if (had_decode_error) {
            rt.last_cmd_ms = now_ms;
            led_health = LedHealth::kYellow;
        } else if ((now_ms - rt.last_cmd_ms) > kCmdTimeoutMs) {
            teleop.axial_dir = 0;
            teleop.turn_dir = 0;
            led_health = LedHealth::kRed;
        }
        set_led_health(led_health);

        float vx_mps = 0.0f;
        float yaw_degps = 0.0f;
        update_command_from_mode(teleop, &vx_mps, &yaw_degps);

        ik.compute(vx_mps, yaw_degps, rt.w_cmd);
        control::Limits::clamp_wheel_commands(
            rt.w_cmd, rt.w_cmd, control::kNumWheels, -control::kWheelSpeedLimitDegPerS,
            control::kWheelSpeedLimitDegPerS);

        if (teleop.axial_dir == 0) {
            stop_all_motors(motors);
        } else {
            apply_open_loop_to_motors(rt.w_cmd, motors);
        }

        if ((rt.last_enc_ms == 0U) || ((now_ms - rt.last_enc_ms) >= kEncoderSamplePeriodMs)) {
            const float dt_enc_s =
                (rt.last_enc_ms == 0U)
                    ? std::max(static_cast<float>(now_ms) * 0.001f, control::kDt)
                    : std::max(static_cast<float>(now_ms - rt.last_enc_ms) * 0.001f, control::kDt);
            wheel_meas_deg_per_s(rt.w_meas, dt_enc_s);
            rt.last_enc_ms = now_ms;
        }

        if ((now_ms - rt.last_print_ms) >= kPrintPeriodMs || got_cmd) {
            rt.last_print_ms = now_ms;
            print_status(now_ms, teleop, rt, vx_mps, yaw_degps);
        }

        HAL_Delay(static_cast<uint32_t>(control::kDt * 1000.0f));
    }
}

extern "C" {
void _init(void) {}
}
 