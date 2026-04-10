#pragma once

#include "config.hpp"
#include "limits.hpp"

namespace control {

/**
 * @Author: Vishnu Duriseti
 * Algebraic IK: (v_cmd, yaw_rate_cmd) -> wheel speed commands.
 * No dynamics, no physics, no state.
 * Output w_cmd[0..3] matches firmware motors[] / motor_bridge_apply order:
 *   [0] LR, [1] LF, [2] RR, [3] RF (see motor_config.h WHEEL_IDX_*).
 * set_geometry r1..r4 are radii [m] for LR, LF, RR, RF respectively.
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
     * v_cmd [m/s] body forward, yaw_rate_cmd [deg/s] (CCW positive from above).
     * If |v_cmd| > 0, yaw is limited so both sides stay forward (or both reverse)
     * — differential arc, not inside-wheel reversal; v_cmd ≈ 0 allows spin.
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
