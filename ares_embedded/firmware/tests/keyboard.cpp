/**
 * @file keyboard.cpp
 * @brief Serial arrow-key teleop (MC33926) + encoder counts and output-shaft revs.
 *
 * Encoder line: LR/LF/RR/RF counts match motors[] / WHEEL_IDX_* (see motor_config.h).
 * Cumulative output-shaft revolutions = sum(encoder deltas) / (ENCODER_COUNTS_PER_REV*gear).
 * Printed as signed revs with 3 decimals (integer math, no %f).
 *
 * Controls over USART3 (115200 8N1):
 *   Primary: W forward, S reverse, A left, D right (case-insensitive), Space or X stop.
 *   Arrows still work if the terminal sends CSI/SS3 sequences (multi-byte).
 */

#include <cstdint>

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#include "encoder_config.h"
#include "encoder_driver.h"
#include "motor_config.h"
#include "motor_driver.h"

#include "uart_print.h"

extern "C" {
void SystemInit(void);
void _init(void);
}

static_assert(MOTOR_COUNT == ENCODER_COUNT,
              "motors[] and encoder_driver must use the same wheel index (0..3)");

/* Saved each ENC print: encoder delta / (ENCODER_COUNTS_PER_REV * gear). */
static float rpm_wheel_lr = 0.f;
static float rpm_wheel_lf = 0.f;
static float rpm_wheel_rr = 0.f;
static float rpm_wheel_rf = 0.f;

typedef enum {
    TELEOP_STOP = 0,
    TELEOP_FORWARD,
    TELEOP_REVERSE,
    TELEOP_LEFT,
    TELEOP_RIGHT,
} teleop_cmd_t;

static const char *teleop_cmd_name(teleop_cmd_t cmd)
{
    switch (cmd) {
        case TELEOP_FORWARD: return "FORWARD";
        case TELEOP_REVERSE: return "REVERSE";
        case TELEOP_LEFT:    return "LEFT";
        case TELEOP_RIGHT:   return "RIGHT";
        case TELEOP_STOP:
        default:             return "STOP";
    }
}

static void apply_teleop_command(motor_driver_t *motors, teleop_cmd_t cmd,
                                 uint8_t duty_move_percent, uint8_t duty_turn_percent)
{
    if (!motors) {
        return;
    }

    switch (cmd) {
        case TELEOP_FORWARD:
            for (int i = 0; i < MOTOR_COUNT; ++i) {
                motor_drive(&motors[i], MOTOR_FORWARD, duty_move_percent);
            }
            break;
        case TELEOP_REVERSE:
            for (int i = 0; i < MOTOR_COUNT; ++i) {
                motor_drive(&motors[i], MOTOR_REVERSE, duty_move_percent);
            }
            break;
        case TELEOP_LEFT:
            motor_drive(&motors[WHEEL_IDX_LR], MOTOR_REVERSE, duty_turn_percent);
            motor_drive(&motors[WHEEL_IDX_LF], MOTOR_REVERSE, duty_turn_percent);
            motor_drive(&motors[WHEEL_IDX_RR], MOTOR_FORWARD, duty_turn_percent);
            motor_drive(&motors[WHEEL_IDX_RF], MOTOR_FORWARD, duty_turn_percent);
            break;
        case TELEOP_RIGHT:
            motor_drive(&motors[WHEEL_IDX_LR], MOTOR_FORWARD, duty_turn_percent);
            motor_drive(&motors[WHEEL_IDX_LF], MOTOR_FORWARD, duty_turn_percent);
            motor_drive(&motors[WHEEL_IDX_RR], MOTOR_REVERSE, duty_turn_percent);
            motor_drive(&motors[WHEEL_IDX_RF], MOTOR_REVERSE, duty_turn_percent);
            break;
        case TELEOP_STOP:
        default:
            for (int i = 0; i < MOTOR_COUNT; ++i) {
                motor_drive(&motors[i], MOTOR_BRAKE, 0);
            }
            break;
    }
}

static uint8_t poll_teleop_command(teleop_cmd_t *out_cmd)
{
    if (!out_cmd) {
        return 0U;
    }

    /* 0=normal, 1=after ESC, 2=CSI ESC [, 3=SS3 ESC O (some terminals) */
    static uint8_t esc_state = 0U;
    uint8_t        c         = 0U;
    uint8_t        got_cmd   = 0U;

    while (uart_try_read_byte(&c)) {
        if (esc_state == 0U) {
            if (c == 0x1BU) {
                esc_state = 1U;
            } else if (c == ' ' || c == '\r' || c == '\n' || c == 'x' || c == 'X') {
                if (c == ' ' || c == 'x' || c == 'X') {
                    *out_cmd = TELEOP_STOP;
                    got_cmd  = 1U;
                }
            } else if (c == 'w' || c == 'W') {
                *out_cmd = TELEOP_FORWARD;
                got_cmd  = 1U;
            } else if (c == 's' || c == 'S') {
                *out_cmd = TELEOP_REVERSE;
                got_cmd  = 1U;
            } else if (c == 'a' || c == 'A') {
                *out_cmd = TELEOP_LEFT;
                got_cmd  = 1U;
            } else if (c == 'd' || c == 'D') {
                *out_cmd = TELEOP_RIGHT;
                got_cmd  = 1U;
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
            }
            continue;
        }

        switch (c) {
            case 'A': *out_cmd = TELEOP_FORWARD; got_cmd = 1U; break;
            case 'B': *out_cmd = TELEOP_REVERSE; got_cmd = 1U; break;
            case 'C': *out_cmd = TELEOP_RIGHT;   got_cmd = 1U; break;
            case 'D': *out_cmd = TELEOP_LEFT;    got_cmd = 1U; break;
            default: break;
        }
        esc_state = 0U;
    }

    return got_cmd;
}

int main(void)
{
    HAL_Init();
    print_init();
    encoder_driver_init(HAL_GetTick());

    static TIM_HandleTypeDef htim_pwm = {};
    motor_timer_init(&htim_pwm, MOTOR_PWM_TIMER, MOTOR_PWM_HZ, MOTOR_TIMER_CLK_HZ);

    static motor_driver_config_t configs[MOTOR_COUNT] = MOTOR_DRIVER_CONFIGS(&htim_pwm);
    static motor_driver_t        motors[MOTOR_COUNT]  = {};

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motor_driver_init(&configs[i], &motors[i]);
    }

    constexpr uint8_t kTeleopDutyPercent      = 30U; /* forward / reverse */
    constexpr uint8_t kTeleopTurnDutyPercent = 10U; /* left / right */
    constexpr uint32_t kPulsePrintPeriodMs = 500U;
    constexpr uint32_t kCmdTimeoutMs       = 60000U;

    teleop_cmd_t active_cmd    = TELEOP_STOP;
    uint32_t     last_cmd_ms   = HAL_GetTick();
    uint32_t     last_pulse_ms = last_cmd_ms;

    apply_teleop_command(motors, active_cmd, kTeleopDutyPercent, kTeleopTurnDutyPercent);
    println("Keyboard teleop ready (USART3 @115200).");
    println(
        "WASD + arrows; Space/X stop. Fwd/rev=%u%% turn=%u%%",
        static_cast<unsigned>(kTeleopDutyPercent),
        static_cast<unsigned>(kTeleopTurnDutyPercent));

    while (true) {
        const uint32_t now_ms = HAL_GetTick();
        teleop_cmd_t   next_cmd = active_cmd;

        if (poll_teleop_command(&next_cmd)) {
            active_cmd = next_cmd;
            last_cmd_ms = now_ms;
            apply_teleop_command(motors, active_cmd, kTeleopDutyPercent, kTeleopTurnDutyPercent);
            println("CMD=%s", teleop_cmd_name(active_cmd));
        }

        if (active_cmd != TELEOP_STOP && (now_ms - last_cmd_ms) > kCmdTimeoutMs) {
            active_cmd = TELEOP_STOP;
            apply_teleop_command(motors, active_cmd, kTeleopDutyPercent, kTeleopTurnDutyPercent);
            println("CMD=STOP (timeout)");
        }

        if ((now_ms - last_pulse_ms) >= kPulsePrintPeriodMs) {
            last_pulse_ms = now_ms;
            const int32_t c_lr =
                encoder_driver_get_and_reset_counts(WHEEL_IDX_LR);
            const int32_t c_lf =
                encoder_driver_get_and_reset_counts(WHEEL_IDX_LF);
            const int32_t c_rr =
                encoder_driver_get_and_reset_counts(WHEEL_IDX_RR);
            const int32_t c_rf =
                encoder_driver_get_and_reset_counts(WHEEL_IDX_RF);

            const float counts_per_output_rev =
                ENCODER_COUNTS_PER_REV * ENCODER_GEAR_RATIO;

           
            rpm_wheel_lr = (static_cast<float>(c_lr) / counts_per_output_rev) * 2.0f * 60.0f;
            rpm_wheel_lf = (static_cast<float>(c_lf) / counts_per_output_rev) * 2.0f * 60.0f;
            rpm_wheel_rr = (static_cast<float>(c_rr) / counts_per_output_rev) * 2.0f * 60.0f;
            rpm_wheel_rf = (static_cast<float>(c_rf) / counts_per_output_rev) * 2.0f * 60.0f;
            

            println(
                "ENC dt=%lums LR=%ld LF=%ld RR=%ld RF=%ld cpr=%lu | rpm_wheel LR=%f LF=%f RR=%f RF=%f",
                static_cast<unsigned long>(kPulsePrintPeriodMs),
                static_cast<long>(c_lr),
                static_cast<long>(c_lf),
                static_cast<long>(c_rr),
                static_cast<long>(c_rf),
                static_cast<unsigned long>(counts_per_output_rev),
                rpm_wheel_lr,
                rpm_wheel_lf,
                rpm_wheel_rr,
                rpm_wheel_rf);
        }

        /* Do not block tens of ms without draining RX: arrow keys are ESC [ x
         * (~3 bytes) and USART can overrun if they arrive during a long sleep. */
        const uint32_t loop_deadline = now_ms + 50U;
        while ((int32_t)(loop_deadline - HAL_GetTick()) > 0) {
            if (poll_teleop_command(&next_cmd)) {
                active_cmd = next_cmd;
                last_cmd_ms = HAL_GetTick();
                apply_teleop_command(motors, active_cmd, kTeleopDutyPercent, kTeleopTurnDutyPercent);
                println("CMD=%s", teleop_cmd_name(active_cmd));
            }
            HAL_Delay(1U);
        }
    }
}

extern "C" {
void _init(void) {}
}
