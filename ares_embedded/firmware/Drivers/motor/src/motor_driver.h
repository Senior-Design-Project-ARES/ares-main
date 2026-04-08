/**
 * @file    motor_driver.h
 * @brief   MC33926 dual H-bridge motor driver — public API
 * @author  Aidan Murray
 * @date    2026-04-06
 * @target  STM32H723ZG + MC33926
 *
 * Pure-C interface for controlling up to MOTOR_COUNT motors through the
 * MC33926 H-bridge IC.  This driver targets the Pololu Dual MC33926 Motor
 * Driver Carrier operating in single-channel mode: both ICs on each carrier
 * board are paralleled (M1ENx+M2ENx tied together, M1/SFx+M2/SFx tied
 * together) to double the continuous current rating per motor.
 *
 * Each motor channel uses:
 *   IN1, IN2   — direction (plain GPIO push-pull)
 *   D2/PWM     — speed (timer alternate-function PWM)
 *   EN         — carrier enable, must be held HIGH for outputs to be active
 *   /SF        — open-drain active-low status flag (fault: overcurrent/overtemp)
 *
 * Typical call sequence:
 *   1. motor_timer_init()   — once per shared PWM timer
 *   2. motor_driver_init()  — once per motor channel
 *   3. motor_drive()        — every control tick
 *   4. motor_get_fault()    — poll /SF as needed
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define MOTOR_COUNT 4

/**
 * MC33926 operating modes (IN1, IN2, PWM truth table).
 *
 *  COAST  : IN1=0, IN2=0, PWM=0  — outputs high-impedance
 *  FORWARD: IN1=1, IN2=0, PWM=duty
 *  REVERSE: IN1=0, IN2=1, PWM=duty
 *  BRAKE  : IN1=1, IN2=1, PWM=0  — low-side brake
 */
typedef enum {
    MOTOR_COAST   = 0,
    MOTOR_FORWARD = 1,
    MOTOR_REVERSE = 2,
    MOTOR_BRAKE   = 3,
} motor_mode_t;

/**
 * Hardware configuration for one motor channel.
 * Fill this struct once at startup; it is consumed by motor_driver_init().
 */
typedef struct {
    /* Direction pins — plain GPIO push-pull, no alternate function */
    GPIO_TypeDef *in1_port;
    uint16_t      in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t      in2_pin;

    /* Speed control — PWM via timer alternate function */
    GPIO_TypeDef      *pwm_port;
    uint16_t           pwm_pin;
    uint8_t            pwm_alternate;  /* e.g. GPIO_AF1_TIM2 */
    TIM_HandleTypeDef *htim_pwm;
    uint32_t           channel_pwm;   /* e.g. TIM_CHANNEL_1 */

    /* Enable pin — push-pull output, held HIGH to enable MC33926 outputs.
     * In single-channel mode, wire M1EN and M2EN together on the carrier
     * and connect to this pin. */
    GPIO_TypeDef *en_port;
    uint16_t      en_pin;

    /* Status flag — input with pull-up; MC33926 asserts LOW on fault
     * (overcurrent, overtemperature).  In single-channel mode, wire
     * M1/SF and M2/SF together on the carrier (both are open-drain). */
    GPIO_TypeDef *sf_port;
    uint16_t      sf_pin;
} motor_driver_config_t;

/**
 * Runtime handle for one motor channel.
 * Populated by motor_driver_init(); treated as opaque by callers.
 */
typedef struct {
    GPIO_TypeDef      *in1_port;
    uint16_t           in1_pin;
    GPIO_TypeDef      *in2_port;
    uint16_t           in2_pin;
    TIM_HandleTypeDef *htim_pwm;
    uint32_t           period;   /* ARR value, used for pulse scaling */
    uint32_t           ch_pwm;
    GPIO_TypeDef      *en_port;
    uint16_t           en_pin;
    GPIO_TypeDef      *sf_port;
    uint16_t           sf_pin;
} motor_driver_t;

/**
 * @brief Configure a PWM timer for motor speed control.
 *
 * Must be called once per timer before motor_driver_init() for any motor
 * that references that timer.  Multiple motors may share a single timer
 * provided they use different channels.
 *
 * The prescaler is chosen as the smallest value that allows ARR to fit in
 * 16 bits (PSC=0 when timer_clk_hz / pwm_hz <= 65535).
 *
 * @param htim          Pointer to an uninitialized TIM_HandleTypeDef.
 * @param instance      Timer peripheral (e.g. TIM8).
 * @param pwm_hz        Desired PWM frequency in Hz (recommend 20000).
 * @param timer_clk_hz  Timer input clock frequency in Hz.
 */
void motor_timer_init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance,
                      uint32_t pwm_hz, uint32_t timer_clk_hz);

/**
 * @brief Initialize one motor channel (GPIO + PWM).
 *
 * Configures IN1/IN2 as push-pull outputs and the PWM pin as an alternate-
 * function output.  Starts the PWM channel with 0% duty.  The timer
 * referenced by config->htim_pwm must already be initialized via
 * motor_timer_init().
 *
 * @param config  Pointer to a fully populated motor_driver_config_t.
 * @param ctx     Pointer to the motor_driver_t handle to be filled.
 */
void motor_driver_init(const motor_driver_config_t *config, motor_driver_t *ctx);

/**
 * @brief Set a motor's direction and speed.
 *
 * Applies the MC33926 truth table: sets IN1/IN2 via HAL_GPIO_WritePin and
 * updates the PWM compare register.  COAST and BRAKE modes ignore
 * duty_percent.  duty_percent is clamped to [0, 100].
 *
 * @param ctx           Initialized motor_driver_t handle.
 * @param mode          Desired operating mode.
 * @param duty_percent  Speed as a percentage of full output (0–100).
 */
void motor_drive(motor_driver_t *ctx, motor_mode_t mode, uint8_t duty_percent);

/**
 * @brief Read the MC33926 status flag for a single motor channel.
 *
 * /SF is asserted LOW by the MC33926 on overcurrent or overtemperature.
 * In single-channel mode, M1/SF and M2/SF must be wired together on the
 * carrier; a fault on either IC will assert this pin.
 *
 * @param ctx  Initialized motor_driver_t handle.
 * @return     1 if a fault is present, 0 if normal.
 */
uint8_t motor_get_fault(const motor_driver_t *ctx);

/**
 * @brief Coast all motors in an array (outputs go high-impedance).
 *
 * @param ctx    Array of initialized motor_driver_t handles.
 * @param count  Number of elements in ctx[].
 */
void motor_all_coast(motor_driver_t ctx[], uint8_t count);

/**
 * @brief Brake all motors in an array (low-side brake).
 *
 * @param ctx    Array of initialized motor_driver_t handles.
 * @param count  Number of elements in ctx[].
 */
void motor_all_brake(motor_driver_t ctx[], uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
