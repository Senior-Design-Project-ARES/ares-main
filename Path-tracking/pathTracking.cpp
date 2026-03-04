/*
  pathTracking.cpp

  Inputs - state (3x1), micropoints (2 x n), linVel (double), stopIF (boolean, int), angVelClamp (double)
  Outputs - Controls (2 x 1)
*/

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

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

Eigen::Vector2d PP_single(const Eigen::Vector3d& state, const Eigen::MatrixXd& micropoints, 
    double linVel, double LA, int stopIF, double angVelClamp)
{
    Eigen::Vector2d controls;

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

        if (angVel > angVelClamp)
        {
            angVel = angVelClamp;
        }
        if (angVel < -angVelClamp)
        {
            angVel = -angVelClamp;
        }

        if(std::abs(alpha) > (45*(M_PI/180)))
        {
            if(alpha > 0)
            {
                angVel = 0.05;
            }
            if(alpha > 0)
            {
                angVel = -0.05;
            }    
            linVel = 0;
        }

        // ------------------ Store Controls in 2x1 Vector ----------------

        controls << linVel, angVel;
}

int main()
{
    // Will be input from GNC
    Eigen::Vector3d state = {1,2,270*M_PI/180};

    // Will be input from pathInterpolation.cpp
    Eigen::MatrixXd micropoints = readWaypoints("micropoints.txt");
    
    // General Inputs
    double linVel = 0.15;
    double LA = 0.5;
    int stopIF = 0;
    double angVelClamp = 1;

    // Declaration of control vec
    Eigen::Vector2d control;

    // Function call
    control = PP_single(state, micropoints, linVel, LA, stopIF, angVelClamp);  
            
    // Display output
    std::cout << "control = [" << control(0) << ", " << control(1) << "]" << std::endl;
}
