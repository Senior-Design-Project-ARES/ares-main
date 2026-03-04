#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

void codeGen_func(
    const Eigen::Vector3d& State,
    const Eigen::MatrixXd& waypoints,
    double& linVel,
    double& angVel,
    int& last_refindex
)

void writeVector(const std::string& filename, const std::vector<double>& data)
{
    std::ofstream file(filename);
    for (double v : data)
        file << v << "\n";
}

std::vector<double> readCSVLine(const std::string& filename)
{
    std::ifstream file(filename);
    std::vector<double> data;
    std::string line;

    if (!file.is_open())
    {
        std::cerr << "Failed to open " << filename << std::endl;
        return data;
    }

    std::getline(file, line);
    std::stringstream ss(line);
    std::string token;

    while (std::getline(ss, token, ','))
        data.push_back(std::stod(token));

    return data;
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


std::vector<double> readVectorFile(const std::string& filename)
{
    std::ifstream file(filename);
    std::vector<double> data;
    double val;

    while (file >> val)
        data.push_back(val);

    return data;
}
