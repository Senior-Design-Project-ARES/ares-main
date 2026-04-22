#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

void codeGen_func(
    const Eigen::Vector3d& State,
    const Eigen::MatrixXd& waypoints,
    double& linVel,
    double& angVel,
    int& last_refindex
)
{
    using namespace Eigen;

    int N = waypoints.rows();
    if (N < 2) { linVel = angVel = 0.0; return; }

    // ---------------- Micropoint Generation ----------------
    int numSamples = 200;
    VectorXd xs(numSamples), ys(numSamples);

    for (int i = 0; i < numSamples; ++i)
    {
        double u = 1.0 + (N - 1) * i / double(numSamples - 1); // 1 -> N

        int idx = std::clamp(int(std::floor(u)) - 1, 0, N - 2);
        double alpha = u - (idx + 1);

        xs(i) = (1.0 - alpha) * waypoints(idx, 0) + alpha * waypoints(idx + 1, 0);
        ys(i) = (1.0 - alpha) * waypoints(idx, 1) + alpha * waypoints(idx + 1, 1);
    }

    MatrixXd points(numSamples, 2);
    points.col(0) = xs;
    points.col(1) = ys;

    // ---------------- Derivatives & Curvature ----------------
    VectorXd dx(numSamples), dy(numSamples);
    VectorXd ddx(numSamples), ddy(numSamples);

    dx.head(numSamples - 1) = xs.tail(numSamples - 1) - xs.head(numSamples - 1);
    dy.head(numSamples - 1) = ys.tail(numSamples - 1) - ys.head(numSamples - 1);
    dx(numSamples - 1) = dx(numSamples - 2);
    dy(numSamples - 1) = dy(numSamples - 2);

    ddx.head(numSamples - 1) = dx.tail(numSamples - 1) - dx.head(numSamples - 1);
    ddy.head(numSamples - 1) = dy.tail(numSamples - 1) - dy.head(numSamples - 1);
    ddx(numSamples - 1) = ddx(numSamples - 2);
    ddy(numSamples - 1) = ddy(numSamples - 2);

    VectorXd K = (dx.array() * ddy.array() - dy.array() * ddx.array()) /
                 (dx.array().square() + dy.array().square()).pow(1.5);

    VectorXd K_norm = (K.array() - K.minCoeff()) / (K.maxCoeff() - K.minCoeff() + 1e-6);
    VectorXd invK = VectorXd::Ones(numSamples) - K_norm;

    double L_min = 0.05, L_max = 0.15;
    VectorXd lookAhead_var = L_min + invK.array() * (L_max - L_min);

    double linV_min = 0.05, linV_max = 0.2;
    VectorXd linVel_vec = linV_min + invK.array() * (linV_max - linV_min);

    // ---------------- Vehicle Position ----------------
    Vector2d backWheel_pos;
    backWheel_pos << State(0) + std::cos(State(2)) * 0.15,
                     State(1) + std::sin(State(2)) * 0.15;

    VectorXd dists(numSamples);
    for (int i = 0; i < numSamples; ++i)
        dists(i) = (points.row(i).transpose() - backWheel_pos).norm();

    // ---------------- Closest Point ----------------
    int refindex = std::clamp(last_refindex, 0, numSamples - 1);
    double minDist = std::numeric_limits<double>::max();

    for (int i = refindex; i < numSamples; ++i)
    {
        if (dists(i) < minDist)
        {
            minDist = dists(i);
            refindex = i;
        }
    }

    last_refindex = refindex;

    // ---------------- Goal Point ----------------
    int validIdx = -1;
    for (int i = refindex; i < numSamples; ++i)
    {
        if (dists(i) >= lookAhead_var(refindex))
        {
            validIdx = i;
            break;
        }
    }
    if (validIdx < 0) validIdx = numSamples - 1;

    validIdx = std::clamp(validIdx, 0, numSamples - 1);

    Vector2d goal = points.row(validIdx);

    // ---------------- Pure Pursuit ----------------
    linVel = linVel_vec(std::clamp(refindex, 0, numSamples - 1));

    double alpha_angle = std::atan2(goal(1) - backWheel_pos(1),
                                    goal(0) - backWheel_pos(0)) - State(2);
    alpha_angle = std::atan2(std::sin(alpha_angle), std::cos(alpha_angle));

    double kappa = 2.0 * std::sin(alpha_angle) / lookAhead_var(std::clamp(refindex, 0, numSamples - 1));
    angVel = linVel * kappa;
}
