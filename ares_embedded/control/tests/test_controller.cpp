/**
 * Host-side SITL test: run MATLAB actuation_controller_sim test cases 1 or 2,
 * log time series to CSV for plotting (e.g. with plot_controller_csv.m).
 *
 * Test case 1: Mixed "city course" (v and psi from knots, smooth transitions).
 * Test case 2: Slalom (constant speed, alternating turns).
 */
#include "config.hpp"
#include "inverse_kinematics.hpp"
#include "limits.hpp"
#include "sugeno_rules.hpp"
#include "wheel_pid.hpp"
#include <cmath>
#include <cstdio>
#include <vector>

namespace {

// Match MATLAB actuation_controller_sim.m
constexpr float kTauW = 0.20f;   // wheel actuator time constant [s]
constexpr float kKAct = 220.0f;   // deg/s per unit
constexpr float kTEnd = 10.0f;
constexpr float kTauGoal = 0.12f; // smooth transition time for profiles

inline float sigma(float x) {
    return 0.5f * (1.0f + std::tanh(x));
}

// profile_from_knots: smooth piecewise from (t_k, y_k) knots; tau = transition time
float profile_from_knots(float t, const float* t_k, const float* y_k, int n, float tau) {
    float out = y_k[0];
    for (int i = 0; i < n; ++i) {
        float dy = (i == 0) ? y_k[0] : (y_k[i] - y_k[i - 1]);
        out += dy * sigma((t - t_k[i]) / tau);
    }
    return out;
}

// Test case 1: Mixed "city course" (same knots as MATLAB)
void eval_trajectory_case1(float t, float& v_goal, float& psi_goal) {
    static const float v_t[]  = {0.0f, 0.8f, 3.2f, 3.8f, 6.5f, 7.2f, 10.0f};
    static const float v_y[]  = {0.0f, 1.2f, 1.2f, 1.6f, 1.6f, 0.9f, 0.9f};
    static const float psi_t[] = {0.0f, 2.0f, 3.0f, 3.7f, 4.6f, 5.6f, 6.4f, 8.2f, 10.0f};
    static const float psi_y[] = {0.0f, 10.0f, 10.0f, 0.0f, -9.0f, -9.0f, 0.0f, -6.0f, -6.0f};
    v_goal   = profile_from_knots(t, v_t, v_y, 7, kTauGoal);
    psi_goal = profile_from_knots(t, psi_t, psi_y, 9, kTauGoal);
}

// Test case 2: Slalom — constant speed, alternating turns (sawtooth with smoothing)
float sawtooth_wave(float t, float period, float duty) {
    float x = 2.0f * std::fmod(t / period, 1.0f) - 1.0f;
    return std::tanh((x - (1.0f - 2.0f * duty)) / 0.15f);
}

void eval_trajectory_case2(float t, float& v_goal, float& psi_goal) {
    v_goal   = 1.3f;
    psi_goal = 12.0f * std::tanh(sawtooth_wave(t, 2.0f, 0.5f) / 0.15f);
}

// First-order wheel dynamics (match MATLAB): w_next = a*w + b*u
void wheel_dynamics_step(const float u[control::kNumWheels], float w_true[control::kNumWheels],
                         float a, float b) {
    for (int i = 0; i < control::kNumWheels; ++i)
        w_true[i] = a * w_true[i] + b * u[i];
}

// Forward kinematics from wheel speeds to body v and psi_dot (match model_plant.m)
void wheel_to_body(const float w[control::kNumWheels], float& v_true, float& psi_true_deg) {
    constexpr float kDeg2Rad = 3.14159265f / 180.0f;
    float v1 = w[0] * kDeg2Rad * control::kRadiusW1;
    float v2 = w[1] * kDeg2Rad * control::kRadiusW2;
    float v3 = w[2] * kDeg2Rad * control::kRadiusW3;
    float v4 = w[3] * kDeg2Rad * control::kRadiusW4;
    float v_front = 0.5f * (v1 + v2);
    float v_rear_mean = 0.5f * (v3 + v4);
    v_true = 0.5f * (v_front + v_rear_mean);
    float track_rear = control::kRearTrack;
    float psi_dot_rad = (v4 - v3) / (track_rear > 1e-6f ? track_rear : 1e-6f);
    psi_true_deg = psi_dot_rad * (180.0f / 3.14159265f);
}

} // namespace

int main(int argc, char* argv[]) {
    int test_case = 1;
    if (argc >= 2)
        test_case = (argv[1][0] == '2') ? 2 : 1;

    control::InverseKinematics ik;
    control::WheelPid pid(control::kKp, control::kKi, control::kKd, control::kDt);
    pid.reset();

    const float dt = control::kDt;
    const int N = static_cast<int>(kTEnd / dt) + 1;

    // Discrete first-order plant (match MATLAB)
    float a = std::exp(-dt / kTauW);
    float b = kKAct * (1.0f - a);

    float w_true[control::kNumWheels] = {0, 0, 0, 0};
    float w_cmd[control::kNumWheels];
    float w_meas[control::kNumWheels];
    float u[control::kNumWheels];

    // Log for CSV (match actuation_controller_sim.m: v/psi goal & true, wheel goal & measured)
    std::vector<float> log_t, log_v_goal, log_psi_goal, log_v_true, log_psi_true;
    std::vector<float> log_w_cmd[4], log_w_meas[4], log_u[4];
    log_t.reserve(static_cast<size_t>(N));
    log_v_goal.reserve(static_cast<size_t>(N));
    log_psi_goal.reserve(static_cast<size_t>(N));
    log_v_true.reserve(static_cast<size_t>(N));
    log_psi_true.reserve(static_cast<size_t>(N));
    for (int i = 0; i < 4; ++i) {
        log_w_cmd[i].reserve(static_cast<size_t>(N));
        log_w_meas[i].reserve(static_cast<size_t>(N));
        log_u[i].reserve(static_cast<size_t>(N));
    }

    for (int k = 0; k < N; ++k) {
        float t = k * dt;
        float v_goal, psi_goal;
        if (test_case == 2)
            eval_trajectory_case2(t, v_goal, psi_goal);
        else
            eval_trajectory_case1(t, v_goal, psi_goal);

        ik.compute(v_goal, psi_goal, w_cmd);
        for (int i = 0; i < control::kNumWheels; ++i)
            w_meas[i] = w_true[i]; // no measurement noise in this test
        pid.step(w_cmd, w_meas, u);
        wheel_dynamics_step(u, w_true, a, b);

        float v_true, psi_true_deg;
        wheel_to_body(w_true, v_true, psi_true_deg);

        log_t.push_back(t);
        log_v_goal.push_back(v_goal);
        log_psi_goal.push_back(psi_goal);
        log_v_true.push_back(v_true);
        log_psi_true.push_back(psi_true_deg);
        for (int i = 0; i < 4; ++i) {
            log_w_cmd[i].push_back(w_cmd[i]);
            log_w_meas[i].push_back(w_meas[i]);
            log_u[i].push_back(u[i]);
        }
    }

    // Write CSV to logs/ (same columns as needed for MATLAB-style plots)
    const char* log_dir = "logs";
    char csv_name[128];
    std::snprintf(csv_name, sizeof(csv_name), "%s/controller_sim_case%d.csv", log_dir, test_case);
    FILE* fp = std::fopen(csv_name, "w");
    if (!fp) {
        std::fprintf(stderr, "Cannot open %s for write (create logs?).\n", csv_name);
        return 1;
    }
    std::fprintf(fp, "t,v_goal,psi_goal,v_true,psi_true,w_cmd_1,w_cmd_2,w_cmd_3,w_cmd_4,w_meas_1,w_meas_2,w_meas_3,w_meas_4,u_1,u_2,u_3,u_4\n");
    for (size_t k = 0; k < log_t.size(); ++k) {
        std::fprintf(fp, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                     log_t[k], log_v_goal[k], log_psi_goal[k], log_v_true[k], log_psi_true[k],
                     log_w_cmd[0][k], log_w_cmd[1][k], log_w_cmd[2][k], log_w_cmd[3][k],
                     log_w_meas[0][k], log_w_meas[1][k], log_w_meas[2][k], log_w_meas[3][k],
                     log_u[0][k], log_u[1][k], log_u[2][k], log_u[3][k]);
    }
    std::fclose(fp);
    std::printf("Test case %d: wrote %s (%zu samples). Run viz to plot.\n", test_case, csv_name, log_t.size());

    return 0;
}
