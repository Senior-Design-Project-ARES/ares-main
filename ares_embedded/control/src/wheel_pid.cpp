#include "wheel_pid.hpp"

/**
 * @Author: Vishnu Duriseti
 * Discrete-time wheel PID. Holds integrator state internally.
 * No actuator dynamics, no saturation (delegate to Limits).
 * Deterministic and allocation-free (fixed 4 wheels).
 */

namespace control {

WheelPid::WheelPid() = default;

WheelPid::WheelPid(float Kp, float Ki, float Kd, float dt)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd), dt_(dt) {}

void WheelPid::set_gains(float Kp, float Ki, float Kd, float dt) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    dt_ = dt;
}

void WheelPid::step(const float w_cmd[kNumWheels],
                    const float w_meas[kNumWheels],
                    float u_out[kNumWheels]) {
    for (int i = 0; i < kNumWheels; ++i) {
        float e = w_cmd[i] - w_meas[i];
        float de = (dt_ > 0.0f) ? (e - e_prev_[i]) / dt_ : 0.0f;
        ei_[i] += e * dt_;
        float u = Kp_ * e + Ki_ * ei_[i] + Kd_ * de;
        u_out[i] = u;
        e_prev_[i] = e;
    }
}

void WheelPid::reset() {
    Limits::reset_integrator(ei_, kNumWheels);
    for (int i = 0; i < kNumWheels; ++i)
        e_prev_[i] = 0.0f;
}

} // namespace control
