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
#include <cstring>
 
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
 
// Binary command format (little-endian):
// magic u16 (0xA55A), axial_vel_m_s f32, turning_rate_deg_s f32, crc16_ccitt u16.
constexpr uint16_t kCmdMagic = 0xA55AU;
 
#pragma pack(push, 1)
struct TeleopCmdFrame
{
    uint16_t magic;
    float    axial_vel_m_s;
    float    turning_rate_deg_s;
    uint16_t crc;
};
#pragma pack(pop)
 
static_assert(sizeof(TeleopCmdFrame) == 12U, "Unexpected TeleopCmdFrame size");
 
uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint32_t i = 0U; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t b = 0U; b < 8U; ++b) {
            crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                                  : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}
 
bool decode_teleop_frame(const uint8_t *raw, float *axial_vel_m_s,
                         float *turning_rate_deg_s)
{
    if (raw == nullptr || axial_vel_m_s == nullptr || turning_rate_deg_s == nullptr) {
        return false;
    }
 
    TeleopCmdFrame frame{};
    std::memcpy(&frame, raw, sizeof(frame));
    if (frame.magic != kCmdMagic) {
        return false;
    }
 
    const uint16_t got_crc = frame.crc;
    frame.crc              = 0U;
    const uint16_t exp_crc =
        crc16_ccitt(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame) - sizeof(frame.crc));
    if (got_crc != exp_crc) {
        return false;
    }
 
    *axial_vel_m_s    = frame.axial_vel_m_s;
    *turning_rate_deg_s = frame.turning_rate_deg_s;
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
    uint32_t enc_us = 0U;
    const uint32_t loop0 = DWT->CYCCNT;
    const uint32_t now = HAL_GetTick();
    const bool take_enc =
        (rt->last_enc_ms == 0U) || ((now - rt->last_enc_ms) >= kEncoderSamplePeriodMs);
    if (take_enc) {
        const float dt_enc_s =
            (rt->last_enc_ms == 0U)
                ? std::max(static_cast<float>(now) * 0.001f, control::kDt)
                : std::max(static_cast<float>(now - rt->last_enc_ms) * 0.001f, control::kDt);
        const uint32_t enc0 = DWT->CYCCNT;
        wheel_meas_deg_per_s_hw(rt->w_meas_hw, dt_enc_s);
        const uint32_t enc1 = DWT->CYCCNT;
        enc_us              = cycles_to_us(enc1 - enc0);
        rt->last_enc_ms     = now;
    }
 
    const uint32_t ctrl0 = DWT->CYCCNT;
    ik->compute(axial_vel_m_s, turning_rate_deg_s, w_cmd);
    control::Limits::clamp_wheel_commands(w_cmd, w_cmd, control::kNumWheels,
                                          -control::kWheelSpeedLimitDegPerS,
                                          control::kWheelSpeedLimitDegPerS);
    pid->step(w_cmd, rt->w_meas_hw, u_hw);
    control::Limits::clamp_wheel_commands(
        u_hw, u_hw, control::kNumWheels, -control::kOutputLimit,
        control::kOutputLimit);
    apply_actuator_to_motors(u_hw, motors);
    const uint32_t ctrl1 = DWT->CYCCNT;
 
    const uint32_t ctrl_us = cycles_to_us(ctrl1 - ctrl0);
    const uint32_t loop_us = cycles_to_us(ctrl1 - loop0);
    print("cmd v=%f yaw=%f | w_cmd: %f, %f, %f, %f", axial_vel_m_s, turning_rate_deg_s,
          w_cmd[0], w_cmd[1], w_cmd[2], w_cmd[3]);
    println(" | w_meas: %f, %f, %f, %f | enc=%lu us ctrl=%lu us loop=%lu us",
            rt->w_meas_hw[0], rt->w_meas_hw[1], rt->w_meas_hw[2], rt->w_meas_hw[3],
            (unsigned long)enc_us, (unsigned long)ctrl_us, (unsigned long)loop_us);
 }
 
void poll_uart_teleop_and_run(float *axial_vel_m_s, float *turning_rate_deg_s)
{
    if (axial_vel_m_s == nullptr || turning_rate_deg_s == nullptr) {
        return;
    }

    static uint8_t rx_buf[sizeof(TeleopCmdFrame)]{};
    static uint8_t idx = 0U;
    uint8_t byte = 0U;
    while (uart_try_read_byte(&byte)) {
        rx_buf[idx++] = byte;
        if (idx < sizeof(rx_buf)) {
            continue;
        }

        float new_v = 0.f;
        float new_yaw = 0.f;
        if (decode_teleop_frame(rx_buf, &new_v, &new_yaw)) {
            *axial_vel_m_s     = new_v;
            *turning_rate_deg_s = new_yaw;
            println("rx cmd: v=%f yaw=%f", *axial_vel_m_s, *turning_rate_deg_s);
            idx = 0U;
            continue;
        }

        // Sliding-window resync for noisy/unaligned streams.
        for (uint8_t i = 1U; i < idx; ++i) {
            rx_buf[i - 1U] = rx_buf[i];
        }
        idx = static_cast<uint8_t>(idx - 1U);
    }
}

 } // namespace
 
 int main(void)
 {
     HAL_Init();
     uart_driver_init_default();
     dwt_cycle_counter_init();

     /* Hold ETH PHY in reset to silence the 50 MHz RMII REFCLK on PA1 (SB57).
      * PC8 = LAN8742A nRST on Nucleo-H723ZG — verify against UM2407 Table 22. */
     __HAL_RCC_GPIOC_CLK_ENABLE();
     GPIO_InitTypeDef phy_rst = {0};
     phy_rst.Pin   = GPIO_PIN_8;
     phy_rst.Mode  = GPIO_MODE_OUTPUT_PP;
     phy_rst.Pull  = GPIO_NOPULL;
     phy_rst.Speed = GPIO_SPEED_FREQ_LOW;
     HAL_GPIO_Init(GPIOC, &phy_rst);
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

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
 
    HAL_Delay(5000U); /* safety delay */
    println("teleop loop start (binary cmd frames over UART)");

    ControlRuntime rt{};
    float          axial_vel_cmd_m_s   = 0.f;
    float          turning_rate_cmd_deg_s = 0.f;

    while (true) {
        poll_uart_teleop_and_run(&axial_vel_cmd_m_s, &turning_rate_cmd_deg_s);
        run(axial_vel_cmd_m_s, turning_rate_cmd_deg_s, &ik, &pid, motors, &rt);
        HAL_Delay(static_cast<uint32_t>(control::kDt * 1000.f));
     }
 }
 
 extern "C" {
 void _init(void) {}
 }
 