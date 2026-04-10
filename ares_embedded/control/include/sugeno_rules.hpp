#pragma once

namespace control {

/**
 * Sugeno-style rule output: gain multipliers from normalized body-level errors.
 * No fuzzy engine; deterministic rules only.
 * Inputs: velocity error [m/s], yaw-rate error [deg/s].
 * Outputs: Kp_mult, Ki_mult, Kd_mult (applied to base PID gains).
 */
struct SugenoGains {
    float Kp_mult{1.0f};
    float Ki_mult{1.0f};
    float Kd_mult{1.0f};
};

/**
 * Compute gain multipliers from body-level errors.
 * Uses normalized errors (internal scale) and soft saturation (tanh-style).
 */
SugenoGains sugeno_gains(float v_err, float psi_err_deg);

} // namespace control
