#include <eigen3/Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

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

    std::vector<double> t(N);
    for (int i = 0; i < N; i++)
        t[i] = i + 1.0;

    std::vector<double> x(N), y(N);
    for (int i = 0; i < N; i++) {
        x[i] = waypoints(i, 0);
        y[i] = waypoints(i, 1);
    }

    std::vector<double> tq(numSamples);
    double step = double(N - 1) / (numSamples - 1);
    for (int i = 0; i < numSamples; i++)
        tq[i] = 1.0 + i * step;

    std::vector<double> xs = cubicSpline(t, x, tq);
    std::vector<double> ys = cubicSpline(t, y, tq);

    Eigen::MatrixXd micro(xs.size(), 2);
    for (size_t i = 0; i < xs.size(); i++) {
        micro(i, 0) = xs[i];
        micro(i, 1) = ys[i];
    }

    return micro;
}

// ---------------- ROS 2 Node ----------------
class PathInterpolatorNode : public rclcpp::Node {
public:
    PathInterpolatorNode() : Node("pathInterp") 
    {
        micro_pub_ = this->create_publisher<nav_msgs::msg::Path>("/micro_waypoints", 10);

        // Get package share directory
        std::string pkg_path = ament_index_cpp::get_package_share_directory("my_package");
        std::string file_path = pkg_path + "/resources/waypoints.txt";  

        Eigen::MatrixXd waypoints = readWaypoints(file_path);
        int numSamples = 100;
        micro_ = micropointsProd(waypoints, numSamples);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PathInterpolatorNode::publishMicroWaypoints, this));
    }

private:
    void publishMicroWaypoints() {
        nav_msgs::msg::Path msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        for (int i = 0; i < micro_.rows(); i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg.header;
            pose.pose.position.x = micro_(i, 0);
            pose.pose.position.y = micro_(i, 1);
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            msg.poses.push_back(pose);
        }

        micro_pub_->publish(msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr micro_pub_;
    Eigen::MatrixXd micro_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    std::cout << "pathInterp started running!" << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathInterpolatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    std::cout << "pathInterp finished!" << std::endl;
    return 0;
}