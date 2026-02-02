/**
 * Host-side SITL test: instantiate controller, feed step commands, print wheel commands.
 * Build with host compiler only; not linked into firmware.
 */
#include "config.hpp"
#include "inverse_kinematics.hpp"
#include "limits.hpp"
#include "sugeno_rules.hpp"
#include "wheel_pid.hpp"
#include <cstdio>

int main() {
    control::InverseKinematics ik;
    control::WheelPid pid(control::kKp, control::kKi, control::kKd, control::kDt);
    pid.reset();

    float v_cmd = 1.0f;       // [m/s]
    float psi_cmd = 0.0f;    // [deg/s]
    float w_cmd[control::kNumWheels];
    float w_meas[control::kNumWheels] = {0.0f, 0.0f, 0.0f, 0.0f};
    float u[control::kNumWheels];

    ik.compute(v_cmd, psi_cmd, w_cmd);
    pid.step(w_cmd, w_meas, u);

    std::printf("v_cmd=%.2f m/s  psi_cmd=%.2f deg/s\n", v_cmd, psi_cmd);
    std::printf("w_cmd [deg/s]: %.2f %.2f %.2f %.2f\n",
                w_cmd[0], w_cmd[1], w_cmd[2], w_cmd[3]);
    std::printf("u (wheel commands): %.4f %.4f %.4f %.4f\n",
                u[0], u[1], u[2], u[3]);

    v_cmd = 1.2f;
    psi_cmd = 10.0f;
    ik.compute(v_cmd, psi_cmd, w_cmd);
    pid.step(w_cmd, w_meas, u);
    std::printf("\nv_cmd=%.2f m/s  psi_cmd=%.2f deg/s\n", v_cmd, psi_cmd);
    std::printf("w_cmd [deg/s]: %.2f %.2f %.2f %.2f\n",
                w_cmd[0], w_cmd[1], w_cmd[2], w_cmd[3]);
    std::printf("u (wheel commands): %.4f %.4f %.4f %.4f\n",
                u[0], u[1], u[2], u[3]);

    if (control::kSugenoEnabled) {
        control::SugenoGains g = control::sugeno_gains(0.2f, 2.0f);
        std::printf("\nSugeno gains: Kp_mult=%.3f Ki_mult=%.3f Kd_mult=%.3f\n",
                   g.Kp_mult, g.Ki_mult, g.Kd_mult);
    }

    return 0;
}
