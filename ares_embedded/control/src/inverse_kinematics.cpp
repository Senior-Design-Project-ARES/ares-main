#include "inverse_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace control {

namespace {
constexpr float kDeg2Rad = 3.14159265f / 180.0f;
constexpr float kRad2Deg = 180.0f / 3.14159265f;
} // namespace

InverseKinematics::InverseKinematics() = default;

void InverseKinematics::set_geometry(float r1, float r2, float r3, float r4,
                                      float rear_track) {
    r1_ = r1;
    r2_ = r2;
    r3_ = r3;
    r4_ = r4;
    rear_track_ = rear_track;
}

void InverseKinematics::compute(float v_cmd, float yaw_rate_cmd_deg,
                                float w_cmd[kNumWheels]) {
    const float psi_dot_rad = yaw_rate_cmd_deg * kDeg2Rad;
    const float half_track  = rear_track_ * 0.5f;

    /* Skid-steer: same speed on each side; left vs right split by yaw rate about
     * vertical axis (positive yaw = CCW from above → right side faster forward).
     * Indices: LR, LF, RR, RF (motor_config.h). */
    const float v_left  = v_cmd - psi_dot_rad * half_track;
    const float v_right = v_cmd + psi_dot_rad * half_track;

    w_cmd[0] = (v_left / r1_) * kRad2Deg;   /* LR */
    w_cmd[1] = (v_left / r2_) * kRad2Deg;   /* LF */
    w_cmd[2] = (v_right / r3_) * kRad2Deg;  /* RR */
    w_cmd[3] = (v_right / r4_) * kRad2Deg;  /* RF */
}

} // namespace control
