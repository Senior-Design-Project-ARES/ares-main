#include "limits.hpp"

namespace control {

float Limits::clamp(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void Limits::clamp_wheel_commands(float* out, const float* cmd, int n,
                                  float lo, float hi) {
    for (int i = 0; i < n; ++i)
        out[i] = clamp(cmd[i], lo, hi);
}

float Limits::clamp_integral(float ei, float e, float Kp, float Ki, float Kd,
                             float de, float u_lo, float u_hi) {
    if (Ki <= 0.0f) return ei;
    float u_prop = Kp * e + Kd * de;
    float u_from_i_lo = (u_lo - u_prop) / Ki;
    float u_from_i_hi = (u_hi - u_prop) / Ki;
    float ei_lo = u_from_i_lo < u_from_i_hi ? u_from_i_lo : u_from_i_hi;
    float ei_hi = u_from_i_lo < u_from_i_hi ? u_from_i_hi : u_from_i_lo;
    return clamp(ei, ei_lo, ei_hi);
}

void Limits::reset_integrator(float* ei, int n) {
    for (int i = 0; i < n; ++i)
        ei[i] = 0.0f;
}

} // namespace control
