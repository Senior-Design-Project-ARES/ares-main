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
Eigen::MatrixXd micropointsProd(const Eigen::MatrixXd& waypoints, int numSamples)
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
    int best_idx = index;

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

    return Eigen::Vector2d(
        micropoints(best_idx, 0),
        micropoints(best_idx, 1)
    );
}


double findClosestWaypointIndex(
    const Eigen::Vector2d& backWheel_pos,
    const Eigen::MatrixXd& micropoints)
{
    double minDistSq = std::numeric_limits<double>::infinity();
    int closestIdx = -1;

    const double x = backWheel_pos(0);
    const double y = backWheel_pos(1);

    for (int i = 0; i < micropoints.rows(); ++i)
    {
        double dx = micropoints(i, 0) - x;
        double dy = micropoints(i, 1) - y;

        double distSq = dx * dx + dy * dy;

        if (distSq < minDistSq)
        {
            minDistSq = distSq;
            closestIdx = i;
        }
    }

    return static_cast<double>(closestIdx);
}

Eigen::Vector4d PP_single(const Eigen::Vector3d& state, const Eigen::MatrixXd& micropoints, 
    double linVel, double LA, int stopIF, double angVelClamp)
{
    Eigen::Vector4d controls;

    if (stopIF == 1)
    {
        controls << 0, 0, 0, 0;
    }
    else
    {
        const Eigen::Vector2d backWheel_pos = {state[0] - cos(state[2]) * 0.15, state[1] - sin(state[2]) * 0.15};

        double closest_idx = findClosestWaypointIndex(backWheel_pos, micropoints);

        Eigen::Vector2d target = findPointAtDistance(micropoints, backWheel_pos, LA, closest_idx);
        
        long double dy = target(1) - backWheel_pos(1);
        long double dx = target(0) - backWheel_pos(0);

        double targ = std::atan2(dy, dx); 
        double alpha = targ - state[2];
        alpha = std::atan2(std::sin(alpha), std::cos(alpha));                   

        double kappa = (2 * std::sin(alpha)) / (std::sqrt(dx*dx + dy*dy));
    
        double angVel = kappa * linVel;

        if(std::abs(alpha) > (45*(M_PI/180)))
        {
            if(alpha > 0)
            {
                angVel = 0.5;
            }
            if(alpha > 0)
            {
                angVel = -0.5;
            }    
            linVel = 0;
        }

        if (angVel > angVelClamp)
        {
            angVel = angVelClamp;
        }
        if (angVel < -angVelClamp)
        {
            angVel = -angVelClamp;
        }

        // ------------------ Store Controls in 3x1 Vector ----------------

        controls << linVel, angVel, dx, dy;
    }

    /*
    std::cout << "Closest Point = [" << microp(closest_idx,0) << ", " << microp(closest_idx,1) << "]" << std::endl;
    std::cout << "Target Point = [" << target(0) << ", " << target(1) << "]" << std::endl;
    std::cout << "Backwheel Position = [" << backWheel_pos(0) << ", " << backWheel_pos(1) << "]" << std::endl;
    std::cout << "Theta = " << state[2] << std::endl;
    std::cout << "Alpha = " << alpha*180/M_PI << std::endl;
    std::cout << "Base = " << std::atan(dy / dx) << std::endl;
    std::cout << "dx, dy = " << dx << ", " << dy << std::endl;
    */

    return controls;
}

Eigen::Vector3d stateCalc(const Eigen::Vector4d& controls, const Eigen::Vector3d& state, double delta_t)
{
    Eigen::Vector3d newState;
    
    newState(2) = state(2) + (delta_t*controls(1));

    if(controls(1) != 0)
    {
        double R = controls(0) / controls(1);

        newState(0) = state(0) + R * (std::sin(newState(2)) - std::sin(state(2)));

        newState(1) = state(1) - R * (std::cos(newState(2)) - std::cos(state(2)));
    }
    else
    {        
        newState(0) = state(0) + std::cos(state(2)) * controls(0) * delta_t;
        newState(1) = state(1) + std::sin(state(2)) * controls(0) * delta_t;
    }

    return newState;
}

int stop_flag(const Eigen::Vector3d& state, const Eigen::MatrixXd& micropoints)
{
    if (micropoints.rows() == 0) {
        // handle empty matrix case
        std::cerr << "Error: micropoints is empty!\n";
        return 0; // or set dist_last = 0 or some safe value
    }

    int last_idx = micropoints.rows() - 1;
    double dist_last = std::sqrt(
        std::pow(state(0) - micropoints(last_idx, 0), 2) +
        std::pow(state(1) - micropoints(last_idx, 1), 2)
    );

    if(dist_last <= 0.3)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int main()
{
    Eigen::Vector3d state = {1,2,180*M_PI/180};

    Eigen::MatrixXd waypoints = readWaypoints("waypoints1.txt");
    double linVel = 0.15;
    double LA = 0.7;
    int numSamples = 100;
    int stopIF = 0;
    double angVelClamp = 0.5;
    double delta_time = 0.1;
    int num_states = 15000;

    Eigen::MatrixXd control_vec(4, num_states); 
    Eigen::MatrixXd state_vec(3, num_states); 
    
    Eigen::Vector4d control;

    // Path interpolation function
    auto micropoints = micropointsProd(waypoints, numSamples);
    writeVector("micropoints.txt", micropoints);

    for(int i = 0; i < num_states; i++)
    {
        //Eigen::Vector2d target(target_vec[2*i], target_vec[2*i+1]);
        
        stopIF = stop_flag(state, micropoints);

        control = PP_single(state, micropoints, linVel, LA, stopIF, angVelClamp);  
        
        control_vec.col(i) = control;

        state_vec.col(i) = state;

        state = stateCalc(control, state, delta_time);
        std::cout << " linVel: " << control(0)
          << " angVel: " << control(1)
          << std::endl;
    }    
    writeVector("Controls_CLA.txt", control_vec);
    writeVector("States_CLA.txt", state_vec);
    
    //control = PP_single(state, waypoints, linVel, LA); 
    //std::cout << "control = [" << control(0) << ", " << control(1) << "]" << std::endl;
}
