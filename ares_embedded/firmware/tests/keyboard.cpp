/**
 * @file keyboard_move_motor.cpp
 * @brief Serial arrow-key teleop for 4 motor drivers + encoder pulse prints.
 * @author Vishnu Duriseti
 * 
 * Controls over USART3 (115200 8N1):
 *   Up    -> forward
 *   Down  -> reverse
 *   Left  -> turn left
 *   Right -> turn right
 *   Space/S -> stop (brake)
 */

#include <cstdint>

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#include "encoder_driver.h"
#include "motor_config.h"
#include "motor_driver.h"

#include "uart_print.h"

extern "C" {
void SystemInit(void);
void _init(void);
}

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

static void apply_teleop_command(motor_driver_t *motors, teleop_cmd_t cmd, uint8_t duty_percent)
{
    if (!motors) {
        return;
    }

    switch (cmd) {
        case TELEOP_FORWARD:
            for (int i = 0; i < MOTOR_COUNT; ++i) {
                motor_drive(&motors[i], MOTOR_FORWARD, duty_percent);
            }
            break;
        case TELEOP_REVERSE:
            for (int i = 0; i < MOTOR_COUNT; ++i) {
                motor_drive(&motors[i], MOTOR_REVERSE, duty_percent);
            }
            break;
        case TELEOP_LEFT:
            motor_drive(&motors[0], MOTOR_REVERSE, duty_percent);
            motor_drive(&motors[2], MOTOR_REVERSE, duty_percent);
            motor_drive(&motors[1], MOTOR_FORWARD, duty_percent);
            motor_drive(&motors[3], MOTOR_FORWARD, duty_percent);
            break;
        case TELEOP_RIGHT:
            motor_drive(&motors[0], MOTOR_FORWARD, duty_percent);
            motor_drive(&motors[2], MOTOR_FORWARD, duty_percent);
            motor_drive(&motors[1], MOTOR_REVERSE, duty_percent);
            motor_drive(&motors[3], MOTOR_REVERSE, duty_percent);
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

    static uint8_t esc_state = 0U;
    uint8_t        c         = 0U;
    uint8_t        got_cmd   = 0U;

    while (uart_try_read_byte(&c)) {
        if (esc_state == 0U) {
            if (c == 0x1BU) {
                esc_state = 1U;
            } else if (c == ' ' || c == 's' || c == 'S') {
                *out_cmd  = TELEOP_STOP;
                got_cmd   = 1U;
            }
            continue;
        }

        if (esc_state == 1U) {
            esc_state = (c == '[') ? 2U : 0U;
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

    constexpr uint8_t  kTeleopDutyPercent  = 25U; // 25% duty cycle
    constexpr uint32_t kPulsePrintPeriodMs = 500U;
    constexpr uint32_t kCmdTimeoutMs       = 400U;

    teleop_cmd_t active_cmd    = TELEOP_STOP;
    uint32_t     last_cmd_ms   = HAL_GetTick();
    uint32_t     last_pulse_ms = last_cmd_ms;

    apply_teleop_command(motors, active_cmd, kTeleopDutyPercent);
    println("Keyboard teleop ready (USART3 @115200).");
    println("Arrows move, Space/S stop. Duty=%u%%", static_cast<unsigned>(kTeleopDutyPercent));

    while (true) {
        uint32_t     now_ms = HAL_GetTick();
        teleop_cmd_t next_cmd = active_cmd;

        if (poll_teleop_command(&next_cmd)) {
            active_cmd = next_cmd;
            last_cmd_ms = now_ms;
            apply_teleop_command(motors, active_cmd, kTeleopDutyPercent);
            println("CMD=%s", teleop_cmd_name(active_cmd));
        }

        if (active_cmd != TELEOP_STOP && (now_ms - last_cmd_ms) > kCmdTimeoutMs) {
            active_cmd = TELEOP_STOP;
            apply_teleop_command(motors, active_cmd, kTeleopDutyPercent);
            println("CMD=STOP (timeout)");
        }

        if ((now_ms - last_pulse_ms) >= kPulsePrintPeriodMs) {
            last_pulse_ms = now_ms;
            int32_t c1 = encoder_driver_get_and_reset_counts(0U);
            int32_t c2 = encoder_driver_get_and_reset_counts(1U);
            int32_t c3 = encoder_driver_get_and_reset_counts(2U);
            int32_t c4 = encoder_driver_get_and_reset_counts(3U);
            println("PULSES d200ms M1=%ld M2=%ld M3=%ld M4=%ld",
                    static_cast<long>(c1),
                    static_cast<long>(c2),
                    static_cast<long>(c3),
                    static_cast<long>(c4));
        }

        HAL_Delay(50U); // sleep for 50ms
    }
}

extern "C" {
void _init(void) {}
}
