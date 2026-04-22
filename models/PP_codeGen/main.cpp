#include <eigen3/Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

/* ============================================================
   Helper functions
   ============================================================ */

// Read a single line CSV file into a vector<double>
std::vector<double> readCSVLine(const std::string& filename)
{
    std::ifstream file(filename);
    std::vector<double> data;
    std::string line;

    if (!file.is_open())
    {
        std::cerr << "Failed to open " << filename << "\n";
        return data;
    }

    std::getline(file, line);
    std::stringstream ss(line);
    std::string token;

    while (std::getline(ss, token, ','))
        data.push_back(std::stod(token));

    return data;
}

// Read waypoints as a Nx2 matrix
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

// Write vector<double> to a text file
void writeVector(const std::string& filename, const std::vector<double>& data)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to write " << filename << "\n";
        return;
    }
    for (double v : data)
        file << v << "\n";
}

/* ============================================================
   Controller function (declaration only)
   ============================================================ */
void codeGen_func(
    const Eigen::Vector3d& State,
    const Eigen::MatrixXd& waypoints,
    double& linVel,
    double& angVel,
    int& last_refindex
);

/* ============================================================
   MAIN
   ============================================================ */
int main()
{
    std::vector<double> rawState = readCSVLine("states.txt");
    Eigen::MatrixXd waypoints = readWaypoints("waypoints.txt");

    if (waypoints.size() == 0)
    {
        std::cerr << "Waypoints input file could not be read.\n";
        return -1;
    }

    if (rawState.empty())
    {
        std::cerr << "State input file could not be read.\n";
        return -1;
    }

    if (rawState.size() % 3 != 0)
    {
        std::cerr << "State file length is not divisible by 3\n";
        return -1;
    }

    int numSteps = rawState.size() / 3;

    std::vector<double> cpp_linVel(numSteps);
    std::vector<double> cpp_angVel(numSteps);

    int last_refindex = 0; // zero-based

    for (int i = 0; i < numSteps; ++i)
    {
        std::vector<double> rawState = readCSVLine("states.txt");
        Eigen::MatrixXd waypoints = readWaypoints("waypoints.txt");

        Eigen::Vector3d State;
        State << rawState[3*i],
                 rawState[3*i + 1],
                 rawState[3*i + 2] * M_PI / 180.0; // deg → rad

        double linVel = 0.0;
        double angVel = 0.0;

        codeGen_func(State, waypoints, linVel, angVel, last_refindex);

        cpp_linVel[i] = linVel;
        cpp_angVel[i] = angVel;
    }

    writeVector("cpp_linVel.txt", cpp_linVel);
    writeVector("cpp_angVel.txt", cpp_angVel);

    std::cout << "C++ results written to cpp_linVel.txt and cpp_angVel.txt\n";

    return 0;
}
