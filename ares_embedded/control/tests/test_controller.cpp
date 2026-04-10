/**
 * Host-side SITL test: run MATLAB actuation_controller_sim test cases 1 or 2,
 * log time series to CSV for plotting (e.g. with plot_controller_csv.m).
 *
 * Test case 1: Mixed "city course" (v and psi from knots, smooth transitions).
 * Test case 2: Slalom (constant speed, alternating turns).
 *
 * Runs two simulations with identical stochastic perturbation:
 *   1. Baseline PID
 *   2. PID + Sugeno fuzzy gain scheduling
 * Outputs: logs/controller_sim_case{N}.csv       (baseline)
 *          logs/controller_sim_case{N}_fuzzy.csv  (Sugeno-enhanced)
 */
#include "ares_control.hpp"
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

namespace {

// ── Plant model (match MATLAB actuation_controller_sim.m) ───────────────────
constexpr float kTauW    = 0.20f;   // wheel actuator time constant [s]
constexpr float kKAct    = 220.0f;  // deg/s per unit
constexpr float kTEnd    = 10.0f;
constexpr float kTauGoal = 0.12f;   // smooth transition time for profiles

// ── Stochastic perturbation ─────────────────────────────────────────────────
constexpr float kMeasNoiseStd    = 2.0f;   // wheel encoder noise σ [deg/s]
constexpr float kProcessNoiseStd = 0.5f;   // plant disturbance σ [deg/s]
constexpr unsigned kRngSeed      = 42u;    // fixed seed for reproducibility

// ── Helpers ─────────────────────────────────────────────────────────────────

inline float sigma(float x) {
    return 0.5f * (1.0f + std::tanh(x));
}

float profile_from_knots(float t, const float* t_k, const float* y_k, int n, float tau) {
    float out = y_k[0];
    for (int i = 0; i < n; ++i) {
        float dy = (i == 0) ? y_k[0] : (y_k[i] - y_k[i - 1]);
        out += dy * sigma((t - t_k[i]) / tau);
    }
    return out;
}

void eval_trajectory_case1(float t, float& v_goal, float& psi_goal) {
    static const float v_t[]   = {0.0f, 0.8f, 3.2f, 3.8f, 6.5f, 7.2f, 10.0f};
    static const float v_y[]   = {0.0f, 1.2f, 1.2f, 1.6f, 1.6f, 0.9f, 0.9f};
    static const float psi_t[] = {0.0f, 2.0f, 3.0f, 3.7f, 4.6f, 5.6f, 6.4f, 8.2f, 10.0f};
    static const float psi_y[] = {0.0f, 10.0f, 10.0f, 0.0f, -9.0f, -9.0f, 0.0f, -6.0f, -6.0f};
    v_goal   = profile_from_knots(t, v_t, v_y, 7, kTauGoal);
    psi_goal = profile_from_knots(t, psi_t, psi_y, 9, kTauGoal);
}

float sawtooth_wave(float t, float period, float duty) {
    float x = 2.0f * std::fmod(t / period, 1.0f) - 1.0f;
    return std::tanh((x - (1.0f - 2.0f * duty)) / 0.15f);
}

void eval_trajectory_case2(float t, float& v_goal, float& psi_goal) {
    v_goal   = 1.3f;
    psi_goal = 12.0f * std::tanh(sawtooth_wave(t, 2.0f, 0.5f) / 0.15f);
}

void wheel_dynamics_step(const float u[control::kNumWheels],
                         float w_true[control::kNumWheels], float a, float b) {
    for (int i = 0; i < control::kNumWheels; ++i)
        w_true[i] = a * w_true[i] + b * u[i];
}

void wheel_to_body(const float w[control::kNumWheels],
                   float& v_out, float& psi_out_deg) {
    constexpr float kDeg2Rad = 3.14159265f / 180.0f;
    /* w index order: LR, LF, RR, RF — match inverse_kinematics / motor_config. */
    const float v_lr = w[0] * kDeg2Rad * control::kRadiusW1;
    const float v_lf = w[1] * kDeg2Rad * control::kRadiusW2;
    const float v_rr = w[2] * kDeg2Rad * control::kRadiusW3;
    const float v_rf = w[3] * kDeg2Rad * control::kRadiusW4;
    v_out = 0.25f * (v_lr + v_lf + v_rr + v_rf);
    const float track_rear = control::kRearTrack;
    const float psi_dot_rad =
        (v_rr - v_lr) / (track_rear > 1e-6f ? track_rear : 1e-6f);
    psi_out_deg = psi_dot_rad * (180.0f / 3.14159265f);
}

// ── Simulation log ──────────────────────────────────────────────────────────

struct SimLog {
    std::vector<float> t, v_goal, psi_goal, v_true, psi_true;
    std::vector<float> w_cmd[4], w_meas[4], u[4];

    void reserve(size_t n) {
        t.reserve(n); v_goal.reserve(n); psi_goal.reserve(n);
        v_true.reserve(n); psi_true.reserve(n);
        for (int i = 0; i < 4; ++i) {
            w_cmd[i].reserve(n); w_meas[i].reserve(n); u[i].reserve(n);
        }
    }
};

// ── Core simulation ─────────────────────────────────────────────────────────

SimLog run_simulation(int test_case, bool use_sugeno, unsigned seed) {
    const float dt = control::kDt;
    const int N = static_cast<int>(kTEnd / dt) + 1;
    const float a = std::exp(-dt / kTauW);
    const float b = kKAct * (1.0f - a);
    const float u_limit = control::kWheelSpeedLimitDegPerS / kKAct;

    control::InverseKinematics ik;
    control::WheelPid pid(control::kKp, control::kKi, control::kKd, dt);
    pid.reset();

    std::mt19937 rng(seed);
    std::normal_distribution<float> meas_dist(0.0f, kMeasNoiseStd);
    std::normal_distribution<float> proc_dist(0.0f, kProcessNoiseStd);

    float w_true[control::kNumWheels] = {0, 0, 0, 0};
    float w_cmd[control::kNumWheels];
    float w_meas[control::kNumWheels];
    float u_out[control::kNumWheels];

    SimLog log;
    log.reserve(static_cast<size_t>(N));

    for (int k = 0; k < N; ++k) {
        float t = k * dt;

        float v_goal, psi_goal;
        if (test_case == 2)
            eval_trajectory_case2(t, v_goal, psi_goal);
        else
            eval_trajectory_case1(t, v_goal, psi_goal);

        for (int i = 0; i < control::kNumWheels; ++i)
            w_meas[i] = w_true[i] + meas_dist(rng);

        if (use_sugeno) {
            float v_meas, psi_meas;
            wheel_to_body(w_meas, v_meas, psi_meas);
            auto g = control::sugeno_gains(v_goal - v_meas, psi_goal - psi_meas);
            pid.set_gains(control::kKp * g.Kp_mult,
                          control::kKi * g.Ki_mult,
                          control::kKd * g.Kd_mult, dt);
        }

        ik.compute(v_goal, psi_goal, w_cmd);
        pid.step(w_cmd, w_meas, u_out);

        control::Limits::clamp_wheel_commands(u_out, u_out, control::kNumWheels,
                                              -u_limit, u_limit);

        wheel_dynamics_step(u_out, w_true, a, b);
        for (int i = 0; i < control::kNumWheels; ++i)
            w_true[i] += proc_dist(rng);

        float v_true, psi_true_deg;
        wheel_to_body(w_true, v_true, psi_true_deg);

        log.t.push_back(t);
        log.v_goal.push_back(v_goal);
        log.psi_goal.push_back(psi_goal);
        log.v_true.push_back(v_true);
        log.psi_true.push_back(psi_true_deg);
        for (int i = 0; i < 4; ++i) {
            log.w_cmd[i].push_back(w_cmd[i]);
            log.w_meas[i].push_back(w_meas[i]);
            log.u[i].push_back(u_out[i]);
        }
    }
    return log;
}

bool write_csv(const char* path, const SimLog& log) {
    FILE* fp = std::fopen(path, "w");
    if (!fp) {
        std::fprintf(stderr, "Cannot open %s for write.\n", path);
        return false;
    }
    std::fprintf(fp, "t,v_goal,psi_goal,v_true,psi_true,"
                     "w_cmd_1,w_cmd_2,w_cmd_3,w_cmd_4,"
                     "w_meas_1,w_meas_2,w_meas_3,w_meas_4,"
                     "u_1,u_2,u_3,u_4\n");
    for (size_t k = 0; k < log.t.size(); ++k) {
        std::fprintf(fp,
            "%.4f,%.4f,%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f,%.4f\n",
            log.t[k], log.v_goal[k], log.psi_goal[k],
            log.v_true[k], log.psi_true[k],
            log.w_cmd[0][k], log.w_cmd[1][k], log.w_cmd[2][k], log.w_cmd[3][k],
            log.w_meas[0][k], log.w_meas[1][k], log.w_meas[2][k], log.w_meas[3][k],
            log.u[0][k], log.u[1][k], log.u[2][k], log.u[3][k]);
    }
    std::fclose(fp);
    return true;
}

} // namespace

int main(int argc, char* argv[]) {
    int test_case = 1;
    if (argc >= 2)
        test_case = (argv[1][0] == '2') ? 2 : 1;

    SimLog baseline = run_simulation(test_case, false, kRngSeed);
    SimLog fuzzy    = run_simulation(test_case, true,  kRngSeed);

    char csv_base[128], csv_fuzzy[128];
    std::snprintf(csv_base,  sizeof(csv_base),  "logs/controller_sim_case%d.csv", test_case);
    std::snprintf(csv_fuzzy, sizeof(csv_fuzzy), "logs/controller_sim_case%d_fuzzy.csv", test_case);

    write_csv(csv_base, baseline);
    write_csv(csv_fuzzy, fuzzy);

    std::printf("Test case %d: wrote %s and %s (%zu samples each).\n"
                "  noise: meas_σ=%.1f deg/s, proc_σ=%.1f deg/s, seed=%u\n",
                test_case, csv_base, csv_fuzzy, baseline.t.size(),
                kMeasNoiseStd, kProcessNoiseStd, kRngSeed);
    return 0;
}
