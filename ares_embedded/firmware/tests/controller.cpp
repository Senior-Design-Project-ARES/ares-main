/**
 * @file controller.cpp
 * @brief Closed-loop wheel PID:
 *
 * Uses ares_control.hpp (IK + WheelPid + Limits). IK wheel order matches
 * motors[] (LR, LF, RR, RF). Control runs every control::kDt; encoder is
 * sampled at 2 Hz (held z.o.h. between samples for PID feedback).
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

#include "uart_print.h"

extern "C" {
void SystemInit(void);
void _init(void);
}

namespace {

/** Enable DWT cycle counter (CPU cycles between reads). Call after clocks are up. */
void dwt_cycle_counter_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t cycles_to_us(uint32_t cycles)
{
    const uint32_t hz = HAL_RCC_GetSysClockFreq();
    return hz ? static_cast<uint32_t>((static_cast<uint64_t>(cycles) * 1000000ULL) /
                                      static_cast<uint64_t>(hz))
              : 0U;
}

/** u in [-1, 1] → duty 0–100 %, sign → direction. */
void apply_actuator_to_motors(const float u_out[control::kNumWheels],
                              motor_driver_t *motors)
{
    for (int i = 0; i < control::kNumWheels; ++i) {
        const float   u    = u_out[i];
        const float   mag  = std::fabs(u) * 100.f;
        const uint8_t duty = static_cast<uint8_t>(mag);

        print("mag: %f | duty: %d ", mag, static_cast<int>(duty));

        if (duty == 0u) {
            motor_drive(&motors[i], MOTOR_BRAKE, 0u);
        } else if (u > 0.f) {
            motor_drive(&motors[i], MOTOR_FORWARD, duty);
        } else {
            motor_drive(&motors[i], MOTOR_REVERSE, duty);
        }
    }
}

/** Encoder sample period [ms] (2 Hz). */
constexpr uint32_t kEncoderSamplePeriodMs = 10U;

/** Output-shaft deg/s from encoder delta over dt_sample_s [s] since last read. */
void wheel_meas_deg_per_s_hw(float w_meas_hw[control::kNumWheels], float dt_sample_s)
{
    const float cpr_motor = static_cast<float>(ENCODER_COUNTS_PER_REV);
    const float gear      = static_cast<float>(ENCODER_GEAR_RATIO);
    const float dt        = std::max(dt_sample_s, control::kDt);

    const int32_t c_lr = encoder_driver_get_and_reset_counts(WHEEL_IDX_LR);
    const int32_t c_lf = encoder_driver_get_and_reset_counts(WHEEL_IDX_LF);
    const int32_t c_rr = -encoder_driver_get_and_reset_counts(WHEEL_IDX_RR);
    const int32_t c_rf = -encoder_driver_get_and_reset_counts(WHEEL_IDX_RF);

    const int32_t counts[4] = {c_lr, c_lf, c_rr, c_rf};

    for (int i = 0; i < control::kNumWheels; ++i) {
        const float motor_rev = static_cast<float>(counts[i]) / cpr_motor;
        const float out_rev   = motor_rev / gear;
        w_meas_hw[i]          = (out_rev * 360.f) / dt;
    }
}

void run_ramp(float v_start_m_s, float v_end_m_s, float yaw_start_deg_s,
              float yaw_end_deg_s, uint32_t duration_ms,
              control::InverseKinematics *ik, control::WheelPid *pid,
              motor_driver_t *motors)
{
    const uint32_t t_start   = HAL_GetTick();
    const uint32_t t_end     = t_start + duration_ms;
    float          w_cmd[control::kNumWheels];
    float          w_meas_hw[control::kNumWheels]{};
    float          u_hw[control::kNumWheels];
    uint32_t       last_enc_ms = 0U;

    while ((int32_t)(t_end - HAL_GetTick()) > 0) {
        const uint32_t now = HAL_GetTick();
        const float    t_s =
            static_cast<float>(now - t_start) * 0.001f;
        const float    t_span_s =
            static_cast<float>(duration_ms) * 0.001f;
        const float alpha =
            (t_span_s > 0.f) ? std::min(1.f, t_s / t_span_s) : 1.f;
        const float v_cmd_m_s =
            v_start_m_s + alpha * (v_end_m_s - v_start_m_s);
        const float yaw_cmd_deg_s =
            yaw_start_deg_s + alpha * (yaw_end_deg_s - yaw_start_deg_s);

        const bool take_enc =
            (last_enc_ms == 0U)
            || ((now - last_enc_ms) >= kEncoderSamplePeriodMs);

        uint32_t enc_us = 0U;
        const uint32_t loop0 = DWT->CYCCNT;

        if (take_enc) {
            const float dt_enc_s =
                (last_enc_ms == 0U)
                    ? std::max(static_cast<float>(now) * 0.001f, control::kDt)
                    : std::max(static_cast<float>(now - last_enc_ms) * 0.001f,
                               control::kDt);

            const uint32_t enc0 = DWT->CYCCNT;
            wheel_meas_deg_per_s_hw(w_meas_hw, dt_enc_s);
            const uint32_t enc1 = DWT->CYCCNT;
            enc_us = cycles_to_us(enc1 - enc0);
            last_enc_ms = now;
        }

        /* IK → PID → motors (includes mag/duty UART inside apply_actuator). */
        const uint32_t ctrl0 = DWT->CYCCNT;
        ik->compute(v_cmd_m_s, yaw_cmd_deg_s, w_cmd);

        control::Limits::clamp_wheel_commands(w_cmd, w_cmd, control::kNumWheels,
                                              -control::kWheelSpeedLimitDegPerS,
                                              control::kWheelSpeedLimitDegPerS);

        pid->step(w_cmd, w_meas_hw, u_hw);
        control::Limits::clamp_wheel_commands(
            u_hw, u_hw, control::kNumWheels, -control::kOutputLimit,
            control::kOutputLimit);

        apply_actuator_to_motors(u_hw, motors);
        const uint32_t ctrl1 = DWT->CYCCNT;

        const uint32_t ctrl_us = cycles_to_us(ctrl1 - ctrl0);
        const uint32_t loop_us = cycles_to_us(ctrl1 - loop0);

        print("v=%f yaw=%f | w_cmd: %f, %f, %f, %f", v_cmd_m_s, yaw_cmd_deg_s,
              w_cmd[0], w_cmd[1], w_cmd[2], w_cmd[3]);
        print(" | w_meas_hw: %f, %f, %f, %f", w_meas_hw[0], w_meas_hw[1], w_meas_hw[2],
              w_meas_hw[3]);
        print(" | u_hw: %f, %f, %f, %f", u_hw[0], u_hw[1], u_hw[2], u_hw[3]);
        println(" | enc: %lu us | ctrl: %lu us | loop: %lu us", enc_us, ctrl_us,
                loop_us);

        HAL_Delay(static_cast<uint32_t>(control::kDt * 1000.f));
    }
}

} // namespace

int main(void)
{
    HAL_Init();
    print_init();
    dwt_cycle_counter_init();
    encoder_driver_init(HAL_GetTick());

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

    // constexpr uint32_t kTestMs       = 10000U;
    // constexpr float    kForwardV     = 2.0f;  /* m/s at start */
    // constexpr float    kReverseV     = -2.0f; /* m/s at end */
    // constexpr float    kRampYawDegS  = 0.f;   /* straight line: yaw = 0 */

    // println("test 1 — straight ramp v: %.2f->%.2f m/s over %lu ms", kForwardV,
    //         kReverseV, static_cast<unsigned long>(kTestMs));

    // run_ramp(kForwardV, kReverseV, kRampYawDegS, kRampYawDegS, kTestMs, &ik, &pid,
    //          motors);

    // motor_all_brake(motors, MOTOR_COUNT);
    // println("pause 1 s");
    // HAL_Delay(1000U);

    // pid.reset();

    constexpr uint32_t kTurnMs       = 10000U;
    constexpr float    kTurnV        = 0.0f; /* m/s, constant */
    constexpr float    kTurnYawDegS  = 35.f;  /* deg/s, constant */

    println("test 2 — turn: v=%.2f m/s, yaw=%.1f deg/s for %lu ms", kTurnV,
            kTurnYawDegS, static_cast<unsigned long>(kTurnMs));
    run_ramp(kTurnV, kTurnV, kTurnYawDegS, kTurnYawDegS, kTurnMs, &ik, &pid, motors);

    motor_all_brake(motors, MOTOR_COUNT);
    println("done — motors brake");

    while (true) {
        HAL_Delay(500U);
    }
}

extern "C" {
void _init(void) {}
}
