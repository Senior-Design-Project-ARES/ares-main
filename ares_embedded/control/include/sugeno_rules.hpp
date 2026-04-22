#pragma once

#include "limits.hpp"

namespace control {

/**
 * Sugeno-style gain multipliers (same Kp,Ki,Kd applied to every wheel in WheelPid).
 */
struct SugenoGains {
    float Kp_mult{1.0f};
    float Ki_mult{1.0f};
    float Kd_mult{1.0f};
};

/**
 * Per-wheel state for splitting wheel speed error into slow (|e| LPF) and fast (|de| LPF)
 * envelopes [deg/s] and [deg/s^2]-like magnitude of de.
 */
struct SugenoWheelErrorState {
    float e_prev[kNumWheels]{};
    float abs_e_lpf[kNumWheels]{};
    float abs_de_lpf[kNumWheels]{};
    bool primed{false};
};

/** Zero filters and derivative memory (e.g. after estop / PID reset). */
inline void sugeno_wheel_error_reset(SugenoWheelErrorState* st)
{
    if (st == nullptr) {
        return;
    }
    *st = SugenoWheelErrorState{};
}

/**
 * Wheel-domain Sugeno: for each wheel, LF/HF metrics → local boost triplets, then
 * average those four triplets into one update (shared gains across wheels).
 * No forward kinematics.
 */
SugenoGains sugeno_gains_from_wheel_errors(SugenoWheelErrorState* st,
                                           const float w_cmd[kNumWheels],
                                           const float w_meas[kNumWheels], float dt);

} // namespace control
