#include <eigen3/Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

struct Point {
    double x;
    double y;
};

// ---------------------- Converts Points to Matrix -------------------------
Eigen::MatrixXd pointsToMatrix(const std::vector<Point>& pts)
{
    Eigen::MatrixXd M(pts.size(), 2);
    for (size_t i = 0; i < pts.size(); ++i)
    {
        M(i, 0) = pts[i].x;
        M(i, 1) = pts[i].y;
    }
    return M;
}

void writeVector(const std::string& filename, const Eigen::MatrixXd& data)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to write " << filename << "\n";
        return;
    }

    for (int i = 0; i < data.rows(); ++i)
    {
        for (int j = 0; j < data.cols(); ++j)
        {
            file << data(i, j);
            if (j < data.cols() - 1)
                file << ",";
        }
        file << "\n";
    }
}

//------------------- Waypoint Reading Function ---------------------
Eigen::MatrixXd readWaypoints(const std::string& filename)
{
    std::ifstream file(filename);
    std::vector<Eigen::Vector2d> wp;

    if (!file.is_open())
    {
        std::cerr << "Failed to open waypoints file: " << filename << "\n";
        return Eigen::MatrixXd();
    }

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty()) continue;

        // Replace commas with spaces
        for (auto &c : line) if (c == ',') c = ' ';

        std::stringstream ss(line);
        double x, y;
        if (ss >> x >> y)
            wp.emplace_back(x, y);
        else
            std::cerr << "Skipping invalid line in waypoints: " << line << "\n";
    }

    Eigen::MatrixXd waypoints(wp.size(), 2);
    for (size_t i = 0; i < wp.size(); ++i)
        waypoints.row(i) = wp[i];

    return waypoints;
}

// ---------------- Cubic spline helper ----------------
std::vector<double> cubicSpline(
    const std::vector<double>& t,
    const std::vector<double>& y,
    const std::vector<double>& tq)
{
    int n = t.size();
    std::vector<double> a(y), b(n), d(n), h(n);
    std::vector<double> alpha(n), c(n+1), l(n+1), mu(n+1), z(n+1);

    for (int i = 0; i < n - 1; i++)
        h[i] = t[i+1] - t[i];

    for (int i = 1; i < n - 1; i++)
        alpha[i] = (3.0/h[i])*(a[i+1]-a[i]) - (3.0/h[i-1])*(a[i]-a[i-1]);

    l[0] = 1.0;
    mu[0] = z[0] = 0.0;

    for (int i = 1; i < n - 1; i++) {
        l[i] = 2.0*(t[i+1]-t[i-1]) - h[i-1]*mu[i-1];
        mu[i] = h[i]/l[i];
        z[i] = (alpha[i]-h[i-1]*z[i-1])/l[i];
    }

    l[n-1] = 1.0;
    z[n-1] = c[n-1] = 0.0;

    for (int j = n - 2; j >= 0; j--) {
        c[j] = z[j] - mu[j]*c[j+1];
        b[j] = (a[j+1]-a[j])/h[j] - h[j]*(c[j+1]+2.0*c[j])/3.0;
        d[j] = (c[j+1]-c[j])/(3.0*h[j]);
    }

    // Evaluate spline
    std::vector<double> result;
    for (double xq : tq) {
        int i = std::min(int(xq) - 1, n - 2);  // convert 1-based → 0-based
        double dx = xq - t[i];
        result.push_back(a[i] + b[i]*dx + c[i]*dx*dx + d[i]*dx*dx*dx);
    }

    return result;
}

// ---------------- Micropoints function ----------------
Eigen::MatrixXd micropoints(const Eigen::MatrixXd& waypoints)
{
    int N = waypoints.rows();

    // MATLAB-style parameter t = 1:N
    std::vector<double> t(N);
    for (int i = 0; i < N; i++)
        t[i] = i + 1.0;

    // Extract x and y
    std::vector<double> x(N), y(N);
    for (int i = 0; i < N; i++) {
        x[i] = waypoints(i, 0);
        y[i] = waypoints(i, 1);
    }

    // Query points (like linspace in MATLAB)
    int numSamples = 500;
    std::vector<double> tq(numSamples);
    double step = double(N - 1) / (numSamples - 1);
    for (int i = 0; i < numSamples; i++)
        tq[i] = 1.0 + i * step;

    // Spline evaluation
    std::vector<double> xs = cubicSpline(t, x, tq);
    std::vector<double> ys = cubicSpline(t, y, tq);

    // Output as Eigen::MatrixXd (numSamples × 2)
    Eigen::MatrixXd micro(xs.size(), 2);
    for (size_t i = 0; i < xs.size(); i++) {
        micro(i, 0) = xs[i];
        micro(i, 1) = ys[i];
    }

    return micro;
}


Eigen::Vector2d findPointAtDistance(
    const Eigen::MatrixXd& micropoints,
    const Eigen::Vector2d& state,
    double desired_dist,
    double index)
{
    double desired_dist2 = desired_dist * desired_dist;

    double best_error = std::numeric_limits<double>::infinity();
    int best_idx = -1;

    for (int i = index; i < micropoints.rows(); ++i)
    {
        double dx = micropoints(i, 0) - state(0);
        double dy = micropoints(i, 1) - state(1);
        double dist2 = dx*dx + dy*dy;

        double error = std::abs(dist2 - desired_dist2);

        if (error < best_error)
        {
            best_error = error;
            best_idx = i;
        }
    }

    return micropoints.row(best_idx);
}

double findClosestWaypointIndex(
    const Eigen::Vector2d& backWheel_pos,
    const Eigen::MatrixXd& waypoints)
{
    double minDistSq = std::numeric_limits<double>::infinity();
    int closestIdx = -1;

    const double x = backWheel_pos(0);
    const double y = backWheel_pos(1);

    for (int i = 0; i < waypoints.rows(); ++i)
    {
        double dx = waypoints(i, 0) - x;
        double dy = waypoints(i, 1) - y;

        double distSq = dx * dx + dy * dy;

        if (distSq < minDistSq)
        {
            minDistSq = distSq;
            closestIdx = i;
        }
    }

    return static_cast<double>(closestIdx);
}

Eigen::Vector2d PP_single(const Eigen::Vector3d& state, const Eigen::MatrixXd& waypoints, 
    double linVel, double LA)
{
    const Eigen::Vector2d backWheel_pos = {state[0] - cos(state[2]) * 0.15, state[1] - sin(state[2]) * 0.15};

    auto micro = micropoints(waypoints);
    writeVector("micropoints.txt", micro);

    Eigen::MatrixXd microp = readWaypoints("micropoints.txt");

    double closest_idx = findClosestWaypointIndex(backWheel_pos, microp);
    Eigen::Vector2d target = findPointAtDistance(microp, backWheel_pos, LA, closest_idx);
    
    long double dy = target(1) - backWheel_posY;
    long double dx = target(0) - backWheel_posX;

    double targ = std::atan2(dy, dx); 
    double alpha = targ - State[2];
    alpha = std::atan2(std::sin(alpha), std::cos(alpha));  ;                   

    double kappa = (2 * std::sin(alpha)) / LA;
    double angVel = kappa * linVel;

    // ------------------ Store Controls in 3x1 Vector ----------------
    Eigen::Vector2d controls;

    std::cout << "Closest Point = [" << microp(closest_idx,0) << ", " << microp(closest_idx,1) << "]" << std::endl;
    std::cout << "Target Point = [" << target(0) << ", " << target(1) << "]" << std::endl;
    std::cout << "Backwheel Position = [" << backWheel_pos(0) << ", " << backWheel_pos(1) << "]" << std::endl;
    std::cout << "Theta = " << state[2] << std::endl;
    std::cout << "Alpha = " << alpha*180/M_PI << std::endl;
    std::cout << "Base = " << std::atan(dy / dx) << std::endl;
    std::cout << "dx, dy = " << dx << ", " << dy << std::endl;

    controls << linVel, angVel;

    return controls;
}

int main()
{
    Eigen::Vector3d state = {1,2,270*M_PI/180};
    Eigen::MatrixXd waypoints = readWaypoints("waypoints.txt");
    double linVel = 0.15;
    double LA = 0.5;

    Eigen::Vector2d control;
    control = PP_single(state, waypoints, linVel, LA);  

    std::cout << "control = [" << control(0) << ", " << control(1) << "]" << std::endl;
}
