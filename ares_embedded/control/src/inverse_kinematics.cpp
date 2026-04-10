#include "inverse_kinematics.hpp"
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
    float psi_dot_rad = yaw_rate_cmd_deg * kDeg2Rad;
    float half_track = rear_track_ * 0.5f;

    float v1 = v_cmd;
    float v2 = v_cmd;
    float v3 = v_cmd - psi_dot_rad * half_track;
    float v4 = v_cmd + psi_dot_rad * half_track;

    w_cmd[0] = (v1 / r1_) * kRad2Deg;
    w_cmd[1] = (v2 / r2_) * kRad2Deg;
    w_cmd[2] = (v3 / r3_) * kRad2Deg;
    w_cmd[3] = (v4 / r4_) * kRad2Deg;
}

} // namespace control
