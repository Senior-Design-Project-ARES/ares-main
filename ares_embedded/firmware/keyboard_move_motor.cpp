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
 *   Space/S -> stop
 */

#include <cstdint>
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio_ex.h"
#include "encoder_driver.h"
#include "uart_print.h"

extern "C" {
    void SystemClock_Config(void);
    void SystemInit(void);
    void ExitRun0Mode(void);
}

typedef struct {
    GPIO_TypeDef *port;
    uint16_t     pin;
    uint8_t      alternate;
} motor_pin_t;

typedef struct {
    motor_pin_t        in1;
    motor_pin_t        in2;
    TIM_HandleTypeDef *htim_in1;
    TIM_HandleTypeDef *htim_in2;
    uint32_t           channel_in1;
    uint32_t           channel_in2;
} motor_driver_config_t;

typedef struct {
    TIM_HandleTypeDef *htim_in1;
    TIM_HandleTypeDef *htim_in2;
    uint32_t           period;
    uint32_t           ch_in1;
    uint32_t           ch_in2;
} motor_driver_t;

typedef enum {
    MOTOR_BRAKE_LOW  = 0,
    MOTOR_FORWARD    = 1,
    MOTOR_REVERSE    = 2,
    MOTOR_BRAKE_HIGH = 3,
} motor_mode_t;

typedef enum {
    TELEOP_STOP = 0,
    TELEOP_FORWARD,
    TELEOP_REVERSE,
    TELEOP_LEFT,
    TELEOP_RIGHT,
} teleop_cmd_t;

static void motor_gpio_clk_enable(GPIO_TypeDef *port)
{
    if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
    else if (port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
}

static void motor_tim_clk_enable(TIM_TypeDef *tim)
{
    if (tim == TIM1) __HAL_RCC_TIM1_CLK_ENABLE();
    else if (tim == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
    else if (tim == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
    else if (tim == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
    else if (tim == TIM8) __HAL_RCC_TIM8_CLK_ENABLE();
}

template <uint32_t PWM_HZ>
static void motor_timer_init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance, uint32_t timer_clk_hz)
{
    static_assert(PWM_HZ >= 10000u && PWM_HZ <= 40000u,
                  "Motor PWM frequency must be between 10 kHz and 40 kHz");

    motor_tim_clk_enable(instance);
    htim->Instance               = instance;
    htim->Init.Prescaler         = 0u;
    htim->Init.Period            = (timer_clk_hz / PWM_HZ) - 1u;
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0u;
    HAL_TIM_PWM_Init(htim);
}

static void motor_driver_init(const motor_driver_config_t *config, motor_driver_t *ctx)
{
    if (!config || !ctx) return;

    motor_gpio_clk_enable(config->in1.port);
    motor_gpio_clk_enable(config->in2.port);

    GPIO_InitTypeDef g = {};
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin       = config->in1.pin;
    g.Alternate = config->in1.alternate;
    HAL_GPIO_Init(config->in1.port, &g);

    g.Pin       = config->in2.pin;
    g.Alternate = config->in2.alternate;
    HAL_GPIO_Init(config->in2.port, &g);

    TIM_OC_InitTypeDef oc = {};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;

    HAL_TIM_PWM_ConfigChannel(config->htim_in1, &oc, config->channel_in1);
    HAL_TIM_PWM_Start(config->htim_in1, config->channel_in1);

    HAL_TIM_PWM_ConfigChannel(config->htim_in2, &oc, config->channel_in2);
    HAL_TIM_PWM_Start(config->htim_in2, config->channel_in2);

    ctx->htim_in1 = config->htim_in1;
    ctx->htim_in2 = config->htim_in2;
    ctx->period   = config->htim_in1->Init.Period;
    ctx->ch_in1   = config->channel_in1;
    ctx->ch_in2   = config->channel_in2;
}

static void motor_drive(motor_driver_t *ctx, motor_mode_t mode, uint8_t duty_percent)
{
    if (!ctx) return;
    if (duty_percent > 100u) duty_percent = 100u;

    uint32_t pulse = (uint32_t)duty_percent * (ctx->period + 1u) / 100u;
    if (pulse > ctx->period) pulse = ctx->period;

    switch (mode) {
        case MOTOR_BRAKE_LOW:
            __HAL_TIM_SET_COMPARE(ctx->htim_in1, ctx->ch_in1, 0);
            __HAL_TIM_SET_COMPARE(ctx->htim_in2, ctx->ch_in2, 0);
            break;
        case MOTOR_FORWARD:
            __HAL_TIM_SET_COMPARE(ctx->htim_in1, ctx->ch_in1, pulse);
            __HAL_TIM_SET_COMPARE(ctx->htim_in2, ctx->ch_in2, 0);
            break;
        case MOTOR_REVERSE:
            __HAL_TIM_SET_COMPARE(ctx->htim_in1, ctx->ch_in1, 0);
            __HAL_TIM_SET_COMPARE(ctx->htim_in2, ctx->ch_in2, pulse);
            break;
        case MOTOR_BRAKE_HIGH:
            __HAL_TIM_SET_COMPARE(ctx->htim_in1, ctx->ch_in1, ctx->period);
            __HAL_TIM_SET_COMPARE(ctx->htim_in2, ctx->ch_in2, ctx->period);
            break;
    }
}

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
    if (!motors) return;

    switch (cmd) {
        case TELEOP_FORWARD:
            for (int i = 0; i < 4; ++i) motor_drive(&motors[i], MOTOR_FORWARD, duty_percent);
            break;
        case TELEOP_REVERSE:
            for (int i = 0; i < 4; ++i) motor_drive(&motors[i], MOTOR_REVERSE, duty_percent);
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
            for (int i = 0; i < 4; ++i) motor_drive(&motors[i], MOTOR_BRAKE_LOW, 0);
            break;
    }
}

static uint8_t poll_teleop_command(teleop_cmd_t *out_cmd)
{
    if (!out_cmd) return 0U;

    static uint8_t esc_state = 0U;
    uint8_t c = 0U;
    uint8_t got_cmd = 0U;

    while (uart_try_read_byte(&c)) {
        if (esc_state == 0U) {
            if (c == 0x1BU) {
                esc_state = 1U;
            } else if (c == ' ' || c == 's' || c == 'S') {
                *out_cmd = TELEOP_STOP;
                got_cmd = 1U;
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

    static TIM_HandleTypeDef htim1 = {};
    static TIM_HandleTypeDef htim2 = {};
    motor_timer_init<20000u>(&htim1, TIM1, 64000000u);
    motor_timer_init<20000u>(&htim2, TIM2, 64000000u);

    /* Same mapping as firmware/main.cpp */
    motor_driver_config_t m1_cfg = {
        .in1 = { GPIOE, GPIO_PIN_9,  GPIO_AF1_TIM1 },
        .in2 = { GPIOA, GPIO_PIN_0,  GPIO_AF1_TIM2 },
        .htim_in1 = &htim1, .htim_in2 = &htim2,
        .channel_in1 = TIM_CHANNEL_1, .channel_in2 = TIM_CHANNEL_1,
    };
    motor_driver_config_t m2_cfg = {
        .in1 = { GPIOE, GPIO_PIN_11, GPIO_AF1_TIM1 },
        .in2 = { GPIOA, GPIO_PIN_1,  GPIO_AF1_TIM2 },
        .htim_in1 = &htim1, .htim_in2 = &htim2,
        .channel_in1 = TIM_CHANNEL_2, .channel_in2 = TIM_CHANNEL_2,
    };
    motor_driver_config_t m3_cfg = {
        .in1 = { GPIOE, GPIO_PIN_13, GPIO_AF1_TIM1 },
        .in2 = { GPIOB, GPIO_PIN_10, GPIO_AF1_TIM2 },
        .htim_in1 = &htim1, .htim_in2 = &htim2,
        .channel_in1 = TIM_CHANNEL_3, .channel_in2 = TIM_CHANNEL_3,
    };
    motor_driver_config_t m4_cfg = {
        .in1 = { GPIOE, GPIO_PIN_14, GPIO_AF1_TIM1 },
        .in2 = { GPIOB, GPIO_PIN_11, GPIO_AF1_TIM2 },
        .htim_in1 = &htim1, .htim_in2 = &htim2,
        .channel_in1 = TIM_CHANNEL_4, .channel_in2 = TIM_CHANNEL_4,
    };

    motor_driver_t motors[4] = {};
    motor_driver_init(&m1_cfg, &motors[0]);
    motor_driver_init(&m2_cfg, &motors[1]);
    motor_driver_init(&m3_cfg, &motors[2]);
    motor_driver_init(&m4_cfg, &motors[3]);

    constexpr uint8_t  kTeleopDutyPercent  = 22U;
    constexpr uint32_t kPulsePrintPeriodMs = 200U;
    constexpr uint32_t kCmdTimeoutMs       = 400U;
    constexpr uint32_t kLoopSleepMs        = 10U;

    teleop_cmd_t active_cmd = TELEOP_STOP;
    uint32_t last_cmd_ms = HAL_GetTick();
    uint32_t last_pulse_ms = last_cmd_ms;

    apply_teleop_command(motors, active_cmd, kTeleopDutyPercent);
    println("Keyboard teleop ready (USART3 @115200).");
    println("Arrows move, Space/S stop. Duty=%u%%", static_cast<unsigned>(kTeleopDutyPercent));

    while (true) {
        uint32_t now_ms = HAL_GetTick();
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

        HAL_Delay(kLoopSleepMs);
    }
}

extern "C" {
    void _init(void)
    {
        /* No C++ global constructors needed */
    }
}
