#include "sugeno_rules.hpp"
#include "config.hpp"
#include <cmath>

/**
 * @Author: Vishnu Duriseti
 * Sugeno-style rule output: gain multipliers from normalized body-level errors.
 * No fuzzy engine; deterministic rules only.
 * Inputs: velocity error [m/s], yaw-rate error [deg/s].
 * Outputs: Kp_mult, Ki_mult, Kd_mult (applied to base PID gains).
 */

namespace control {

namespace {
float tanh_approx(float x) {
    if (x < -3.0f) return -1.0f;
    if (x > 3.0f) return 1.0f;
    float t = std::exp(-2.0f * std::fabs(x));
    return x >= 0.0f ? (1.0f - t) / (1.0f + t) : -(1.0f - t) / (1.0f + t);
}
} // namespace

SugenoGains sugeno_gains(float v_err, float psi_err_deg) {
    float Ev = std::fabs(v_err) / kVErrorScale;
    float Epsi = std::fabs(psi_err_deg) / kPsiErrorScale;

    float boostP = 1.0f + 0.4f * tanh_approx(Ev / 0.5f + Epsi / 5.0f);
    float boostI = 1.0f + 0.3f * tanh_approx(0.2f - Ev);
    float boostD = 1.0f + 0.4f * tanh_approx((Ev + Epsi) * 0.5f);

    SugenoGains g;
    g.Kp_mult = boostP;
    g.Ki_mult = boostI;
    g.Kd_mult = boostD;
    return g;
}

} // namespace control
