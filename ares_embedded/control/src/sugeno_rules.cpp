#include "sugeno_rules.hpp"
#include "config.hpp"
#include <cmath>

/**
 * @Author: Vishnu Duriseti
 * Sugeno-style rule output: gain multipliers from per-wheel speed error split.
 * LF: low-pass |e|, HF: low-pass |de| with e = w_cmd - w_meas [deg/s].
 * Each wheel gets a local (Kp_mult, Ki_mult, Kd_mult) candidate; the firmware uses
 * one PID gain set for all wheels, so we average the four candidates then clamp.
 */

namespace control {

namespace {
float tanh_approx(float x)
{
    if (x < -3.0f) {
        return -1.0f;
    }
    if (x > 3.0f) {
        return 1.0f;
    }
    float t = std::exp(-2.0f * std::fabs(x));
    return x >= 0.0f ? (1.0f - t) / (1.0f + t) : -(1.0f - t) / (1.0f + t);
}

/** Unclamped multipliers from normalized LF/HF activity for one wheel. */
void raw_boosts(float n_low, float n_high, float* boost_p, float* boost_i, float* boost_d)
{
    /* LF: push P/I harder on persistent |e|. HF: add D and P; pull I to limit windup. */
    *boost_p = 1.0f + 0.52f * tanh_approx(n_low) + 0.14f * tanh_approx(n_high);
    *boost_i = 1.0f + 0.42f * tanh_approx(n_low) - 0.34f * tanh_approx(n_high);
    *boost_d = 1.0f + 0.58f * tanh_approx(n_high) - 0.07f * tanh_approx(n_low);
}
} // namespace

SugenoGains sugeno_gains_from_wheel_errors(SugenoWheelErrorState* st,
                                           const float w_cmd[kNumWheels],
                                           const float w_meas[kNumWheels], float dt)
{
    SugenoGains g{};
    if (st == nullptr || w_cmd == nullptr || w_meas == nullptr) {
        return g;
    }

    const float dt_safe  = (dt > 1e-9f) ? dt : kDt;
    const float inv_dt   = 1.0f / dt_safe;
    const float alpha_lf = dt_safe / (kSugenoLfTauS + dt_safe);
    const float alpha_hf = dt_safe / (kSugenoHfTauS + dt_safe);

    float sum_p = 0.0f;
    float sum_i = 0.0f;
    float sum_d = 0.0f;

    for (int i = 0; i < kNumWheels; ++i) {
        const float e  = w_cmd[i] - w_meas[i];
        const float de = st->primed ? (e - st->e_prev[i]) * inv_dt : 0.0f;
        st->e_prev[i] = e;

        const float ae  = std::fabs(e);
        const float ade = std::fabs(de);

        if (!st->primed) {
            st->abs_e_lpf[i]  = ae;
            st->abs_de_lpf[i] = ade;
        } else {
            st->abs_e_lpf[i] += alpha_lf * (ae - st->abs_e_lpf[i]);
            st->abs_de_lpf[i] += alpha_hf * (ade - st->abs_de_lpf[i]);
        }

        const float n_low =
            (kSugenoWheelErrLfScale > 1e-9f) ? (st->abs_e_lpf[i] / kSugenoWheelErrLfScale) : 0.0f;
        const float n_high = (kSugenoWheelErrHfScale > 1e-9f)
                                 ? (st->abs_de_lpf[i] / kSugenoWheelErrHfScale)
                                 : 0.0f;

        float bp = 1.0f;
        float bi = 1.0f;
        float bd = 1.0f;
        raw_boosts(n_low, n_high, &bp, &bi, &bd);
        sum_p += bp;
        sum_i += bi;
        sum_d += bd;
    }

    st->primed = true;

    constexpr float inv_n = 1.0f / static_cast<float>(kNumWheels);
    g.Kp_mult = Limits::clamp(sum_p * inv_n, kSugenoKpMultLo, kSugenoKpMultHi);
    g.Ki_mult = Limits::clamp(sum_i * inv_n, kSugenoKiMultLo, kSugenoKiMultHi);
    g.Kd_mult = Limits::clamp(sum_d * inv_n, kSugenoKdMultLo, kSugenoKdMultHi);
    return g;
}

} // namespace control
