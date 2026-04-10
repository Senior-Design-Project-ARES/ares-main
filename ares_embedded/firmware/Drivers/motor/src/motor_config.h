/**
 * @file    motor_config.h
 * @brief   MC33926 hardware pin assignment table — STM32H723ZG Nucleo
 * @author  Aidan Murray
 * @date    2026-04-06
 * @target  STM32H723ZG + 4× Pololu Dual MC33926 Motor Driver Carrier
 *
 * Hardware configuration: 4 carrier boards, each in single-channel mode.
 * Both MC33926 ICs on each carrier are paralleled for double current:
 *   — M1EN + M2EN tied together  → driven by one EN GPIO (active high)
 *   — M1/SF + M2/SF tied together → read by one /SF GPIO (active low, open-drain)
 *   — M1IN1 + M2IN1 tied together → driven by one IN1 GPIO
 *   — M1IN2 + M2IN2 tied together → driven by one IN2 GPIO
 *   — M1D2  + M2D2  tied together → driven by one PWM pin
 *
 * This file is the single source of truth for physical wiring.  All GPIO
 * port/pin/AF selections live here; motor_driver.c and motor_bridge.c
 * contain no board-specific literals.
 *
 * ── HOW TO USE ────────────────────────────────────────────────────────────
 *
 *   #include "motor_config.h"
 *   #include "motor_driver.h"
 *
 *   static TIM_HandleTypeDef g_htim_pwm = {0};
 *   motor_timer_init(&g_htim_pwm, MOTOR_PWM_TIMER,
 *                    MOTOR_PWM_HZ, MOTOR_TIMER_CLK_HZ);
 *
 *   static motor_driver_t g_motors[MOTOR_COUNT] = {0};
 *   for (int i = 0; i < MOTOR_COUNT; i++) {
 *       motor_driver_init(&g_motor_configs[i], &g_motors[i]);
 *   }
 *
 * ── CONSTRAINTS (do not reassign these resources) ─────────────────────────
 *
 *   TIM1  — M2 encoder (hardware quadrature, PE9/PE11)
 *   TIM3  — M1 encoder (hardware quadrature, PC6/CN7-1, PC7/CN7-11)
 *   TIM4  — M3 encoder (hardware quadrature, PB6/PB7)
 *   TIM5  — M4 encoder (hardware quadrature, PA0/PA1)
 *   PB8   — I2C1 SCL (IMU + INA260)
 *   PB9   — I2C1 SDA (IMU + INA260)
 *   PD8   — USART3 TX (ST-LINK debug console)
 *   PD9   — USART3 RX (ST-LINK debug console)
 *
 * ── STATUS: VERIFIED — direction GPIOs confirmed from UM2407 Table 22
 *            morpho pinout and team spreadsheet. PWM pins verified from
 *            DS13313 AF table. ────────────────────────────────────────────
 * ─────────────────────────────────────────────────────────────────────────
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "motor_driver.h"

/*
 * Logical wheel index: use the same value for motors[i] and encoder channel i
 * (encoder_config.h g_encoder_hw[i], TIM assignment in the banner above).
 *
 *   i=0  M1  LR (left-rear)
 *   i=1  M2  LF (left-front)
 *   i=2  M3  RR (right-rear)
 *   i=3  M4  RF (right-front)
 */
#define WHEEL_IDX_LR 0U
#define WHEEL_IDX_LF 1U
#define WHEEL_IDX_RR 2U
#define WHEEL_IDX_RF 3U

#if MOTOR_COUNT != 4
#error "WHEEL_IDX_* assumes four wheels (see encoder_config.h ENCODER_COUNT)."
#endif

/* ── PWM timer ────────────────────────────────────────────────────────────
 *
 * TIM2 is on APB1L and has 4 independent channels — one per motor.
 * It does not conflict with any of the four encoder timers (TIM1/3/4/5).
 *
 * TIM2 primary pins for CH1/CH2 (PA0/PA1) are taken by the M4 encoder.
 * The alternate mappings below avoid all reserved pins:
 *   TIM2_CH1 → PA15 (AF1)  ← verified DS13313
 *   TIM2_CH2 → PB3  (AF1)  ← verified DS13313
 *   TIM2_CH3 → PB10 (AF1)  ← verified DS13313
 *   TIM2_CH4 → PB11 (AF1)  ← verified DS13313
 *
 * Note: TIM2 is a general-purpose timer — no MOE (Main Output Enable) bit
 * required, unlike advanced-control timers (TIM1, TIM8, TIM15–17).
 */
#define MOTOR_PWM_TIMER     TIM2

/* PWM frequency: 20 kHz puts switching noise above audible range (MC33926
 * datasheet recommends > 10 kHz for ultrasonic operation). */
#define MOTOR_PWM_HZ        20000u

/* APB1 timer clock = 96 MHz. SYSCLK=192MHz, AHB/2=96MHz, APB1/2=48MHz,
 * timer x2 rule = 96 MHz. Verified from SystemClock_Config() in this repo. */
#define MOTOR_TIMER_CLK_HZ  96000000u

/* ── Enable and Status Flag GPIO assignments ──────────────────────────────
 *
 * One EN pin and one /SF pin per carrier board.
 * EN  : push-pull output, driven HIGH by motor_driver_init().
 * /SF : input with pull-up, active-low fault indicator.
 *
 * VERIFY THESE AGAINST YOUR ACTUAL PHYSICAL WIRING before building.
 * Pin choices below are free from all other peripheral conflicts on the
 * Nucleo-H723ZG (not used by encoders, UART, I2C, SWD, LEDs, or PWM).
 */

/* Motor 1 — LR (left-rear) */
#define M1_EN_PORT   GPIOE
#define M1_EN_PIN    GPIO_PIN_2   /* CN10 pin 25 */
#define M1_SF_PORT   GPIOG
#define M1_SF_PIN    GPIO_PIN_0   /* CN11 pin 55 */

/* Motor 2 — LF (left-front) */
#define M2_EN_PORT   GPIOE
#define M2_EN_PIN    GPIO_PIN_3   /* CN10 pin 27 */
#define M2_SF_PORT   GPIOG
#define M2_SF_PIN    GPIO_PIN_1   /* CN11 pin 57 */

/* Motor 3 — RR (right-rear) */
#define M3_EN_PORT   GPIOE
#define M3_EN_PIN    GPIO_PIN_4   /* CN10 pin 29 */
#define M3_SF_PORT   GPIOG
#define M3_SF_PIN    GPIO_PIN_2   /* CN10 pin 55 */

/* Motor 4 — RF (right-front) */
#define M4_EN_PORT   GPIOE
#define M4_EN_PIN    GPIO_PIN_5   /* CN10 pin 31 */
#define M4_SF_PORT   GPIOG
#define M4_SF_PIN    GPIO_PIN_3   /* CN10 pin 57 */

/* ── Direction GPIO assignments ───────────────────────────────────────────
 *
 * Each motor needs two push-pull GPIO pins (IN1, IN2); no alternate
 * function is required.  Pins confirmed from UM2407 Table 22 (morpho
 * connector pinout) and team allocation spreadsheet.
 */

/* Motor 1 — LR (left-rear) */
#define M1_IN1_PORT  GPIOC
#define M1_IN1_PIN   GPIO_PIN_11  /* CN11 pin 1  */
#define M1_IN2_PORT  GPIOC
#define M1_IN2_PIN   GPIO_PIN_10  /* CN11 pin 2  */

/* Motor 2 — LF (left-front) */
#define M2_IN1_PORT  GPIOD
#define M2_IN1_PIN   GPIO_PIN_2  /* CN11 pin 3  */
#define M2_IN2_PORT  GPIOC
#define M2_IN2_PIN   GPIO_PIN_12   /* CN11 pin 4  */

/* Motor 3 — RR (right-rear) */
#define M3_IN1_PORT  GPIOF
#define M3_IN1_PIN   GPIO_PIN_6   /* CN11 pin 9  */
#define M3_IN2_PORT  GPIOF
#define M3_IN2_PIN   GPIO_PIN_7   /* CN11 pin 11 */

/* Motor 4 — RF (right-front) */
#define M4_IN1_PORT  GPIOD
#define M4_IN1_PIN   GPIO_PIN_4   /* CN11 pin 39 */
#define M4_IN2_PORT  GPIOD
#define M4_IN2_PIN   GPIO_PIN_5   /* CN11 pin 41 */

/* ── PWM pin assignments ──────────────────────────────────────────────────
 *
 * All four channels use TIM2 (AF1). Primary CH1/CH2 pins (PA0/PA1) are
 * occupied by the M4 encoder; alternate mappings are used instead.
 * All four pins verified against DS13313 Table 13.
 */

/* Motor 1 — LR: TIM2_CH1 → PA15 (AF1) — CN11 pin 17
 * PA15 is also JTDI — if JTAG debugging is needed (not just SWD), use
 * PA5 (AF1) instead.  SWD (which ST-LINK uses) does not require PA15. */
#define M1_PWM_PORT      GPIOA
#define M1_PWM_PIN       GPIO_PIN_15
#define M1_PWM_AF        GPIO_AF1_TIM2
#define M1_PWM_CHANNEL   TIM_CHANNEL_1

/* Motor 2 — LF: TIM2_CH2 → PB3 (AF1) — CN7 pin 15 */
#define M2_PWM_PORT      GPIOB
#define M2_PWM_PIN       GPIO_PIN_3
#define M2_PWM_AF        GPIO_AF1_TIM2
#define M2_PWM_CHANNEL   TIM_CHANNEL_2

/* Motor 3 — RR: TIM2_CH3 → PB10 (AF1) — CN10 pin 32 (right-rear) */
#define M3_PWM_PORT      GPIOB
#define M3_PWM_PIN       GPIO_PIN_10
#define M3_PWM_AF        GPIO_AF1_TIM2
#define M3_PWM_CHANNEL   TIM_CHANNEL_3

/* Motor 4 — RF: TIM2_CH4 → PB11 (AF1) — CN10 pin 34 */
#define M4_PWM_PORT      GPIOB
#define M4_PWM_PIN       GPIO_PIN_11
#define M4_PWM_AF        GPIO_AF1_TIM2
#define M4_PWM_CHANNEL   TIM_CHANNEL_4

/* ── Compile-time config table ────────────────────────────────────────────
 *
 * This macro expands to a brace-initializer for motor_driver_config_t[4].
 * The single shared timer handle (htim_pwm) is passed as a pointer argument
 * because it must be allocated in the caller's translation unit.
 *
 * Usage:
 *   static TIM_HandleTypeDef g_htim_pwm = {0};
 *   static motor_driver_config_t configs[MOTOR_COUNT] =
 *       MOTOR_DRIVER_CONFIGS(&g_htim_pwm);
 */
#define MOTOR_DRIVER_CONFIGS(htim_ptr)          \
{                                               \
    /* M1 — LR */                               \
    {                                           \
        .in1_port      = M1_IN1_PORT,           \
        .in1_pin       = M1_IN1_PIN,            \
        .in2_port      = M1_IN2_PORT,           \
        .in2_pin       = M1_IN2_PIN,            \
        .pwm_port      = M1_PWM_PORT,           \
        .pwm_pin       = M1_PWM_PIN,            \
        .pwm_alternate = M1_PWM_AF,             \
        .htim_pwm      = (htim_ptr),            \
        .channel_pwm   = M1_PWM_CHANNEL,        \
        .en_port       = M1_EN_PORT,            \
        .en_pin        = M1_EN_PIN,             \
        .sf_port       = M1_SF_PORT,            \
        .sf_pin        = M1_SF_PIN,             \
    },                                          \
    /* M2 — LF */                               \
    {                                           \
        .in1_port      = M2_IN1_PORT,           \
        .in1_pin       = M2_IN1_PIN,            \
        .in2_port      = M2_IN2_PORT,           \
        .in2_pin       = M2_IN2_PIN,            \
        .pwm_port      = M2_PWM_PORT,           \
        .pwm_pin       = M2_PWM_PIN,            \
        .pwm_alternate = M2_PWM_AF,             \
        .htim_pwm      = (htim_ptr),            \
        .channel_pwm   = M2_PWM_CHANNEL,        \
        .en_port       = M2_EN_PORT,            \
        .en_pin        = M2_EN_PIN,             \
        .sf_port       = M2_SF_PORT,            \
        .sf_pin        = M2_SF_PIN,             \
    },                                          \
    /* M3 — RR */                               \
    {                                           \
        .in1_port      = M3_IN1_PORT,           \
        .in1_pin       = M3_IN1_PIN,            \
        .in2_port      = M3_IN2_PORT,           \
        .in2_pin       = M3_IN2_PIN,            \
        .pwm_port      = M3_PWM_PORT,           \
        .pwm_pin       = M3_PWM_PIN,            \
        .pwm_alternate = M3_PWM_AF,             \
        .htim_pwm      = (htim_ptr),            \
        .channel_pwm   = M3_PWM_CHANNEL,        \
        .en_port       = M3_EN_PORT,            \
        .en_pin        = M3_EN_PIN,             \
        .sf_port       = M3_SF_PORT,            \
        .sf_pin        = M3_SF_PIN,             \
    },                                          \
    /* M4 — RF */                               \
    {                                           \
        .in1_port      = M4_IN1_PORT,           \
        .in1_pin       = M4_IN1_PIN,            \
        .in2_port      = M4_IN2_PORT,           \
        .in2_pin       = M4_IN2_PIN,            \
        .pwm_port      = M4_PWM_PORT,           \
        .pwm_pin       = M4_PWM_PIN,            \
        .pwm_alternate = M4_PWM_AF,             \
        .htim_pwm      = (htim_ptr),            \
        .channel_pwm   = M4_PWM_CHANNEL,        \
        .en_port       = M4_EN_PORT,            \
        .en_pin        = M4_EN_PIN,             \
        .sf_port       = M4_SF_PORT,            \
        .sf_pin        = M4_SF_PIN,             \
    },                                          \
}

#endif /* MOTOR_CONFIG_H */
