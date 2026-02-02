#pragma once

#include "config.hpp"
#include "limits.hpp"

namespace control {

/**
 * Algebraic IK: (v_cmd, yaw_rate_cmd) -> wheel speed commands.
 * No dynamics, no physics, no state.
 * Wheel order: 1=FR, 2=FL, 3=RL, 4=RR (omni front, normal rear).
 */
class InverseKinematics {
public:
    InverseKinematics();

    /**
     * Set geometry (radii [m], rear track [m]). If not called, config defaults used.
     */
    void set_geometry(float r1, float r2, float r3, float r4, float rear_track);

    /**
     * Compute wheel speed commands [deg/s] from body-level commands.
     * v_cmd [m/s], yaw_rate_cmd [deg/s].
     * Result written to w_cmd[0..3].
     */
    void compute(float v_cmd, float yaw_rate_cmd_deg, float w_cmd[kNumWheels]);

private:
    float r1_{kRadiusW1};
    float r2_{kRadiusW2};
    float r3_{kRadiusW3};
    float r4_{kRadiusW4};
    float rear_track_{kRearTrack};
};

} // namespace control
