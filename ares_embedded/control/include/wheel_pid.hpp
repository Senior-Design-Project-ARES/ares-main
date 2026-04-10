#pragma once

#include "config.hpp"
#include "limits.hpp"

namespace control {

/**
 * Discrete-time wheel PID. Holds integrator state internally.
 * No actuator dynamics, no saturation (delegate to Limits).
 * Deterministic and allocation-free (fixed 4 wheels).
 */
class WheelPid {
public:
    WheelPid();
    WheelPid(float Kp, float Ki, float Kd, float Kf, float dt);

    /**
     * Set gains and sample period.
     */
    void set_gains(float Kp, float Ki, float Kd, float Kf, float dt);

    /**
     * One step: w_cmd and w_meas are wheel speed commands and measurements [deg/s].
     * Output u is written to out[0..3]; same units as your actuator input.
     * Caller applies Limits::clamp_wheel_commands and anti-windup if desired.
     */
    void step(const float w_cmd[kNumWheels], const float w_meas[kNumWheels],
              float u_out[kNumWheels]);

    /**
     * Reset integrator state.
     */
    void reset();

    float Kp() const { return Kp_; }
    float Ki() const { return Ki_; }
    float Kd() const { return Kd_; }
    float Kf() const { return Kf_; }
    float dt() const { return dt_; }

private:
    float Kp_{kKp};
    float Ki_{kKi};
    float Kd_{kKd};
    float Kf_{kKf};
    float dt_{kDt};
    float ei_[kNumWheels]{};
    float e_prev_[kNumWheels]{};
};

} // namespace control
