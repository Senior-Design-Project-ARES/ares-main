/**
 * @file    motor_bridge.c
 * @brief   Bridge layer: control library output → MC33926 motor driver
 * @author  Aidan Murray
 * @date    2026-04-06
 * @target  STM32H723ZG + MC33926
 *
 * Implements the mapping from the signed floating-point actuator commands
 * produced by WheelPid::step() to motor_drive() calls on the MC33926 driver.
 * No dynamic allocation; module state is a single static struct.
 */

#include "motor_bridge.h"
#include <math.h>   /* fabsf */

/* ── Module state ─────────────────────────────────────────────────────────── */

/* Default configuration values (see motor_bridge.h for units/meaning) */
#define BRIDGE_DEFAULT_U_MAX    4.54f
#define BRIDGE_DEFAULT_DEADBAND 0.01f

static motor_bridge_config_t g_config = {
    .u_max    = BRIDGE_DEFAULT_U_MAX,
    .deadband = BRIDGE_DEFAULT_DEADBAND,
};

static uint8_t g_initialised = 0u;

/* ── Public API ──────────────────────────────────────────────────────────── */

void motor_bridge_init(const motor_bridge_config_t *config)
{
    if (config != NULL) {
        /* Accept caller's values only if they are sane */
        g_config.u_max    = (config->u_max    > 0.0f) ? config->u_max    : BRIDGE_DEFAULT_U_MAX;
        g_config.deadband = (config->deadband >= 0.0f) ? config->deadband : BRIDGE_DEFAULT_DEADBAND;
    } else {
        g_config.u_max    = BRIDGE_DEFAULT_U_MAX;
        g_config.deadband = BRIDGE_DEFAULT_DEADBAND;
    }
    g_initialised = 1u;
}

void motor_bridge_apply(const float u_out[4], motor_driver_t motors[4])
{
    /* Auto-init with defaults if motor_bridge_init() was never called */
    if (!g_initialised) {
        motor_bridge_init(NULL);
    }

    for (int i = 0; i < 4; ++i) {
        float u   = u_out[i];
        float mag = fabsf(u);

        if (mag < g_config.deadband) {
            /* Below deadband — brake to resist disturbances at standstill */
            motor_drive(&motors[i], MOTOR_BRAKE, 0u);
            continue;
        }

        /* Scale magnitude to 0–100 duty, clamp at ceiling */
        uint32_t duty32 = (uint32_t)((mag / g_config.u_max) * 100.0f);
        uint8_t  duty   = (duty32 > 100u) ? 100u : (uint8_t)duty32;

        if (u > 0.0f) {
            motor_drive(&motors[i], MOTOR_FORWARD, duty);
        } else {
            motor_drive(&motors[i], MOTOR_REVERSE, duty);
        }
    }
}
