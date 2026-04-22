#pragma once

/**
 * Unified public header for the ARES wheel-speed controller.
 *
 * Include this from low-level firmware instead of the individual headers:
 *
 *   #include "ares_control.hpp"
 *
 * This pulls in:
 *   - config.hpp          (controller parameters and geometry)
 *   - limits.hpp          (saturation and integrator helpers)
 *   - inverse_kinematics.hpp (v, yaw-rate -> wheel speeds)
 *   - wheel_pid.hpp       (per-wheel PID controller)
 *   - sugeno_rules.hpp    (optional Sugeno-style gain scheduling)
 */

#include "config.hpp"
#include "limits.hpp"
#include "inverse_kinematics.hpp"
#include "wheel_pid.hpp"
#include "sugeno_rules.hpp"

