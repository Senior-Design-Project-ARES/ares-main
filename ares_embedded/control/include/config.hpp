#pragma once

/**
 * Central controller parameters. Constants only; no logic.
 * Derived from MATLAB design (actuation_controller_sim.m, test.m).
 */

namespace control {

/// Inner-loop sample period [s]
constexpr float kDt = 0.01f;

/// Wheel PID gains (analytical design: rise time, settling time, damping)
constexpr float kKp = 0.5f;
constexpr float kKi = 2.5f;
constexpr float kKd = 0.0f;

/// Wheel speed limit [deg/s] (command and output)
constexpr float kWheelSpeedLimitDegPerS = 500.0f;

/// Actuator output limit (same units as PID output; scale to your DAC/PWM)
constexpr float kOutputLimit = 1.0f;

/// IK geometry: wheel radii [m] (order: FR, FL, RL, RR)
constexpr float kRadiusW1 = 0.15f;
constexpr float kRadiusW2 = 0.15f;
constexpr float kRadiusW3 = 0.15f;
constexpr float kRadiusW4 = 0.15f;

/// Rear track: distance between rear wheels [m] (d_wheel3_cg + d_wheel4_cg)
constexpr float kRearTrack = 0.70f;

/// Sugeno supervisor: enable (1) or disable (0)
constexpr int kSugenoEnabled = 0;

/// Normalization scales for Sugeno (body-level errors)
constexpr float kVErrorScale = 1.5f;   // [m/s]
constexpr float kPsiErrorScale = 12.0f; // [deg/s]

} // namespace control
