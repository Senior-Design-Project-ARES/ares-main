/**
 * @file    motor_driver.c
 * @brief   MC33926 dual H-bridge motor driver — implementation
 * @author  Aidan Murray
 * @date    2026-04-06
 * @target  STM32H723ZG + MC33926
 *
 * Implements the public API declared in motor_driver.h.  All hardware
 * access goes through STM32 HAL; no register-level manipulation is used
 * here so that the logic remains portable across STM32H7 variants.
 *
 * Direction (IN1/IN2) is set via HAL_GPIO_WritePin.
 * Speed (D2/PWM) is set via __HAL_TIM_SET_COMPARE for zero-overhead
 * compare-register updates after initial channel configuration.
 */

#include "motor_driver.h"

/* ── Internal helpers ────────────────────────────────────────────────────── */

/**
 * Enable the GPIO peripheral clock for a given port.
 * Called before HAL_GPIO_Init to satisfy the HAL requirement that the
 * clock be enabled before touching any register in that port.
 */
static void gpio_clk_enable(GPIO_TypeDef *port)
{
    if      (port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
    else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
    else if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
    else if (port == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); }
    else if (port == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); }
    else if (port == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); }
}

/**
 * Enable the timer peripheral clock for a given TIM instance.
 * Only timers that could plausibly carry PWM for motor control are listed.
 */
static void tim_clk_enable(TIM_TypeDef *tim)
{
    if      (tim == TIM2)  { __HAL_RCC_TIM2_CLK_ENABLE();  }
    else if (tim == TIM8)  { __HAL_RCC_TIM8_CLK_ENABLE();  } /* TIM8 — not currently used, kept for future flexibility */
    else if (tim == TIM12) { __HAL_RCC_TIM12_CLK_ENABLE(); }
    else if (tim == TIM15) { __HAL_RCC_TIM15_CLK_ENABLE(); }
    else if (tim == TIM16) { __HAL_RCC_TIM16_CLK_ENABLE(); }
    else if (tim == TIM17) { __HAL_RCC_TIM17_CLK_ENABLE(); }
    /* TIM1,3,4,5 are reserved for encoders — not listed intentionally */
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void motor_timer_init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance,
                      uint32_t pwm_hz, uint32_t timer_clk_hz)
{
    if (!htim || !instance || pwm_hz == 0u) { return; }

    tim_clk_enable(instance);

    /*
     * Choose the smallest prescaler such that ARR fits in 16 bits.
     *
     *   ARR = (timer_clk_hz / ((PSC+1) * pwm_hz)) - 1
     *
     * Start with PSC=0.  If the raw count exceeds 0xFFFF, increment PSC
     * until it fits.  This keeps duty-cycle resolution as high as possible.
     */
    uint32_t psc = 0u;
    uint32_t arr;
    do {
        arr = (timer_clk_hz / ((psc + 1u) * pwm_hz));
        if (arr > 0u) { arr -= 1u; }
        if (arr <= 0xFFFFu) { break; }
        psc++;
    } while (psc < 0xFFFFu);

    htim->Instance               = instance;
    htim->Init.Prescaler         = psc;
    htim->Init.Period            = arr;
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0u;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    HAL_TIM_PWM_Init(htim);

    /* TIM1, TIM8, TIM15, TIM16, TIM17 are advanced-control timers whose
     * outputs are disabled by default until the Main Output Enable (MOE)
     * bit in the BDTR register is set.  Without this, PWM pins stay low. */
    __HAL_TIM_MOE_ENABLE(htim);
}

void motor_driver_init(const motor_driver_config_t *config, motor_driver_t *ctx)
{
    if (!config || !ctx) { return; }

    /* ── Direction pins: push-pull GPIO output, no alternate function ── */
    gpio_clk_enable(config->in1_port);
    gpio_clk_enable(config->in2_port);

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = config->in1_pin;
    HAL_GPIO_Init(config->in1_port, &gpio);
    HAL_GPIO_WritePin(config->in1_port, config->in1_pin, GPIO_PIN_RESET);

    gpio.Pin = config->in2_pin;
    HAL_GPIO_Init(config->in2_port, &gpio);
    HAL_GPIO_WritePin(config->in2_port, config->in2_pin, GPIO_PIN_RESET);

    /* ── PWM pin: alternate-function push-pull ── */
    gpio_clk_enable(config->pwm_port);

    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = config->pwm_alternate;
    gpio.Pin       = config->pwm_pin;
    HAL_GPIO_Init(config->pwm_port, &gpio);

    /* ── Configure and start PWM channel at 0% duty ── */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0u;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(config->htim_pwm, &oc, config->channel_pwm);
    HAL_TIM_PWM_Start(config->htim_pwm, config->channel_pwm);

    /* ── Enable pin: push-pull output, asserted HIGH immediately ── */
    gpio_clk_enable(config->en_port);
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin   = config->en_pin;
    HAL_GPIO_Init(config->en_port, &gpio);
    HAL_GPIO_WritePin(config->en_port, config->en_pin, GPIO_PIN_SET);

    /* ── Status flag: input with pull-up (/SF is active-low, open-drain) ── */
    gpio_clk_enable(config->sf_port);
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin   = config->sf_pin;
    HAL_GPIO_Init(config->sf_port, &gpio);

    /* ── Populate runtime context ── */
    ctx->in1_port = config->in1_port;
    ctx->in1_pin  = config->in1_pin;
    ctx->in2_port = config->in2_port;
    ctx->in2_pin  = config->in2_pin;
    ctx->htim_pwm = config->htim_pwm;
    ctx->period   = config->htim_pwm->Init.Period;
    ctx->ch_pwm   = config->channel_pwm;
    ctx->en_port  = config->en_port;
    ctx->en_pin   = config->en_pin;
    ctx->sf_port  = config->sf_port;
    ctx->sf_pin   = config->sf_pin;
}

void motor_drive(motor_driver_t *ctx, motor_mode_t mode, uint8_t duty_percent)
{
    if (!ctx) { return; }

    if (duty_percent > 100u) { duty_percent = 100u; }

    /* Scale duty_percent → timer compare value.
     * pulse = duty_percent * (ARR + 1) / 100, clamped to ARR. */
    uint32_t pulse = (uint32_t)duty_percent * (ctx->period + 1u) / 100u;
    if (pulse > ctx->period) { pulse = ctx->period; }

    switch (mode) {
        case MOTOR_COAST:
            /* IN1=0, IN2=0, PWM=0 — outputs float high-impedance */
            HAL_GPIO_WritePin(ctx->in1_port, ctx->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ctx->in2_port, ctx->in2_pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(ctx->htim_pwm, ctx->ch_pwm, 0u);
            break;

        case MOTOR_FORWARD:
            /* IN1=1, IN2=0, PWM=duty */
            HAL_GPIO_WritePin(ctx->in1_port, ctx->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ctx->in2_port, ctx->in2_pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(ctx->htim_pwm, ctx->ch_pwm, pulse);
            break;

        case MOTOR_REVERSE:
            /* IN1=0, IN2=1, PWM=duty */
            HAL_GPIO_WritePin(ctx->in1_port, ctx->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ctx->in2_port, ctx->in2_pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(ctx->htim_pwm, ctx->ch_pwm, pulse);
            break;

        case MOTOR_BRAKE:
            /* IN1=1, IN2=1, PWM=0 — low-side brake */
            HAL_GPIO_WritePin(ctx->in1_port, ctx->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ctx->in2_port, ctx->in2_pin, GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(ctx->htim_pwm, ctx->ch_pwm, 0u);
            break;

        default:
            /* Unknown mode — coast for safety */
            HAL_GPIO_WritePin(ctx->in1_port, ctx->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ctx->in2_port, ctx->in2_pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(ctx->htim_pwm, ctx->ch_pwm, 0u);
            break;
    }
}

uint8_t motor_get_fault(const motor_driver_t *ctx)
{
    if (!ctx) { return 0u; }
    /* /SF is active-low: GPIO_PIN_RESET means the MC33926 has asserted a fault */
    return (HAL_GPIO_ReadPin(ctx->sf_port, ctx->sf_pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

void motor_all_coast(motor_driver_t ctx[], uint8_t count)
{
    for (uint8_t i = 0u; i < count; ++i) {
        motor_drive(&ctx[i], MOTOR_COAST, 0u);
    }
}

void motor_all_brake(motor_driver_t ctx[], uint8_t count)
{
    for (uint8_t i = 0u; i < count; ++i) {
        motor_drive(&ctx[i], MOTOR_BRAKE, 0u);
    }
}
