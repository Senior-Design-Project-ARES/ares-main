#pragma once

/**
 * @author: Vishnu Duriseti
 * Central controller parameters. 
 * Derived from MATLAB design (actuation_controller_sim.m, test.m).
 */

namespace control {

/// Inner-loop sample period [s]
constexpr float kDt = 0.01f;

/// Wheel PID gains (analytical design: rise time, settling time, damping)
// constexpr float kKp = 0.0045f;
// constexpr float kKi = 0.0152f;
// constexpr float kKd = 0.0005f;

constexpr float kKp = 0.00001f;
constexpr float kKi = 0.0001f;
constexpr float kKd = 0.00000f;
constexpr float kKf = 0.0008f;

/// Wheel speed limit [deg/s] (command and output)
constexpr float kWheelSpeedLimitDegPerS = 2000.0f;

/// Actuator output limit (same units as PID output; scale to your DAC/PWM)
constexpr float kOutputLimit = 1.0f;

/// IK geometry: wheel radii [m] in LR, LF, RR, RF order (same as motors[] / inverse_kinematics)
constexpr float kRadiusW1 = 0.15f;
constexpr float kRadiusW2 = 0.15f;
constexpr float kRadiusW3 = 0.15f;
constexpr float kRadiusW4 = 0.15f;

/// Rear track: distance between rear wheels [m] (d_wheel3_cg + d_wheel4_cg)
constexpr float kRearTrack = 0.70f;

/// Sugeno supervisor: enable (1) or disable (0)
constexpr int kSugenoEnabled = 1;

/// LPF time constants for wheel-error split [s]
constexpr float kSugenoLfTauS = 0.14f; // slightly faster LF envelope
constexpr float kSugenoHfTauS = 0.028f; // faster HF tracking → snappier D/P from jitter

/// Smaller scales → same physical error maps to larger n → more aggressive scheduling
constexpr float kSugenoWheelErrLfScale  = 52.0f;
constexpr float kSugenoWheelErrHfScale  = 2600.0f;

/// After averaging per-wheel Sugeno candidates, clamp shared multipliers (wider = more swing)
constexpr float kSugenoKpMultLo = 0.52f;
constexpr float kSugenoKpMultHi = 1.68f;
constexpr float kSugenoKiMultLo = 0.38f;
constexpr float kSugenoKiMultHi = 1.58f;
constexpr float kSugenoKdMultLo = 0.48f;
constexpr float kSugenoKdMultHi = 1.82f;

} // namespace control
