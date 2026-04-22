/**
 * @file    motor_bridge.h
 * @brief   Bridge layer: control library output → MC33926 motor driver
 * @author  Aidan Murray
 * @date    2026-04-06
 * @target  STM32H723ZG + MC33926
 *
 * Connects the floating-point signed actuator commands produced by
 * WheelPid::step() (in ares_embedded/control/) to the MC33926 motor driver
 * API.  Neither the control library nor the motor driver is modified; this
 * module owns the mapping between the two.
 *
 * Expected signal range from WheelPid::step():
 *   u_out[i] ∈ [−4.54, +4.54]  (= ±kWheelSpeedLimitDegPerS / kKAct)
 *   Positive → forward, negative → reverse.
 *
 * Mapping:
 *   |u| < deadband         → MOTOR_BRAKE   (prevent chatter at zero)
 *   u ≥  deadband          → MOTOR_FORWARD, duty = |u| / u_max × 100
 *   u ≤ −deadband          → MOTOR_REVERSE, duty = |u| / u_max × 100
 *   duty clamped to [0, 100]
 */

#ifndef MOTOR_BRIDGE_H
#define MOTOR_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_driver.h"

/**
 * Configuration for the bridge layer.
 * Pass NULL to motor_bridge_init() to use built-in defaults.
 */
typedef struct {
    float u_max;    /**< Saturation limit for u_out magnitude. Default: 4.54f */
    float deadband; /**< |u| below this → brake. Default: 0.01f              */
} motor_bridge_config_t;

/**
 * @brief Initialize the bridge with the given configuration.
 *
 * Stores the configuration in a module-level static struct.
 * Call once before the control loop begins.
 *
 * @param config  Pointer to config, or NULL for defaults
 *                (u_max=4.54, deadband=0.01).
 */
void motor_bridge_init(const motor_bridge_config_t *config);

/**
 * @brief Apply signed control outputs to the motor driver.
 *
 * Maps each element of u_out[4] to a motor_drive() call according to
 * sign and magnitude.  Intended to be called every control tick
 * immediately after WheelPid::step().
 *
 * Array index `i` is passed straight to `motors[i]` (no reordering). Hardware
 * corners (motor_config.h / encoders): [0]=LR, [1]=LF, [2]=RR, [3]=RF
 * (L/R = side, F/R = front/rear axle). InverseKinematics / WheelPid use the
 * same index order as `motors[]`.
 *
 * @param u_out   Signed actuator commands from WheelPid::step() (length 4).
 * @param motors  Initialized motor_driver_t array (length 4).
 */
void motor_bridge_apply(const float u_out[4], motor_driver_t motors[4]);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_BRIDGE_H */
