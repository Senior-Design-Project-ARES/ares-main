#pragma once

namespace control {

/// Number of wheels (fixed for this rover)
constexpr int kNumWheels = 4;

/**
 * Output saturation, integrator anti-windup, and reset utilities.
 * No physics; pure clamping and state reset.
 */
class Limits {
public:
    /**
     * Saturate a scalar to [lo, hi].
     */
    static float clamp(float x, float lo, float hi);

    /**
     * Saturate wheel command array to [lo, hi] per element.
     */
    static void clamp_wheel_commands(float* out, const float* cmd, int n,
                                    float lo, float hi);

    /**
     * Anti-windup: clamp integrator so that (Kp*e + Ki*ei + Kd*de) would not
     * exceed [u_lo, u_hi]. Back-calculate max allowed integral given u_lim.
     * ei_out is written in place; returns clamped value for caller to store.
     */
    static float clamp_integral(float ei, float e, float Kp, float Ki, float Kd,
                               float de, float u_lo, float u_hi);

    /**
     * Reset integrator (and optional derivative state) to zero.
     */
    static void reset_integrator(float* ei, int n);
};

} // namespace control
