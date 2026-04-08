/**
 * @file    motor_config.h
 * @brief   MC33926 hardware pin assignment table — STM32H723ZG Nucleo
 * @author  Aidan Murray
 * @date    2026-04-06
 * @target  STM32H723ZG + MC33926
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
 *   TIM3  — M1 encoder (hardware quadrature, PC6/PC7)
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

/* SYSCLK = 64 MHz (HSI default, no PLL). APB1 prescaler = /1, so TIM2 clock
 * = 64 MHz. Verified from system_stm32h7xx.c and consistent with Vishnu's
 * motor_timer_init<20000u>(&htim1, TIM1, 64000000u). If PLL is configured
 * later, this value must be updated to match. */
#define MOTOR_TIMER_CLK_HZ  64000000u

/* ── Direction GPIO assignments ───────────────────────────────────────────
 *
 * Each motor needs two push-pull GPIO pins (IN1, IN2); no alternate
 * function is required.  Pins confirmed from UM2407 Table 22 (morpho
 * connector pinout) and team allocation spreadsheet.
 */

/* Motor 1 — FR (front-right) */
#define M1_IN1_PORT  GPIOC
#define M1_IN1_PIN   GPIO_PIN_10  /* CN11 pin 1  */
#define M1_IN2_PORT  GPIOC
#define M1_IN2_PIN   GPIO_PIN_11  /* CN11 pin 2  */

/* Motor 2 — FL (front-left) */
#define M2_IN1_PORT  GPIOC
#define M2_IN1_PIN   GPIO_PIN_12  /* CN11 pin 3  */
#define M2_IN2_PORT  GPIOD
#define M2_IN2_PIN   GPIO_PIN_2   /* CN11 pin 4  */

/* Motor 3 — RL (rear-left) */
#define M3_IN1_PORT  GPIOF
#define M3_IN1_PIN   GPIO_PIN_6   /* CN11 pin 9  */
#define M3_IN2_PORT  GPIOF
#define M3_IN2_PIN   GPIO_PIN_7   /* CN11 pin 11 */

/* Motor 4 — RR (rear-right) */
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

/* Motor 1 — FR: TIM2_CH1 → PA15 (AF1)
 * PA15 is also JTDI — if JTAG debugging is needed (not just SWD), use
 * PA5 (AF1) instead.  SWD (which ST-LINK uses) does not require PA15. */
#define M1_PWM_PORT      GPIOA
#define M1_PWM_PIN       GPIO_PIN_15
#define M1_PWM_AF        GPIO_AF1_TIM2
#define M1_PWM_CHANNEL   TIM_CHANNEL_1

/* Motor 2 — FL: TIM2_CH2 → PB3 (AF1) */
#define M2_PWM_PORT      GPIOB
#define M2_PWM_PIN       GPIO_PIN_3
#define M2_PWM_AF        GPIO_AF1_TIM2
#define M2_PWM_CHANNEL   TIM_CHANNEL_2

/* Motor 3 — RL: TIM2_CH3 → PB10 (AF1) */
#define M3_PWM_PORT      GPIOB
#define M3_PWM_PIN       GPIO_PIN_10
#define M3_PWM_AF        GPIO_AF1_TIM2
#define M3_PWM_CHANNEL   TIM_CHANNEL_3

/* Motor 4 — RR: TIM2_CH4 → PB11 (AF1) */
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
    /* M1 — FR */                               \
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
    },                                          \
    /* M2 — FL */                               \
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
    },                                          \
    /* M3 — RL */                               \
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
    },                                          \
    /* M4 — RR */                               \
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
    },                                          \
}

#endif /* MOTOR_CONFIG_H */
