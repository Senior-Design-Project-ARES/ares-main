#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

// Finds closest waypoint
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

// Finds point at lookahead distance
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

int stop_flag(const Eigen::Vector3d& state, const Eigen::MatrixXd& micropoints, double stopDist)
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

    if(dist_last <= stopDist)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// Pure Pursuit controller
Eigen::Vector2d PP_single(
    const Eigen::Vector3d& state,
    const Eigen::MatrixXd& micropoints,
    double linVel,
    double LA,
    int stopIF,
    double angVelClamp,
    double turnRate)
{
    Eigen::Vector2d controls;

    if (stopIF == 1)
    {
        controls << 0, 0;
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
            if(alpha < 0)
            {
                angVel = -turnRate;
            }
            if(alpha > 0)
            {
                angVel = turnRate;
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

        controls << linVel, angVel;
    }

    return controls;
}

// ---------------- ROS2  NODE -------------------

class PathTrackingNode : public rclcpp::Node
{
    private:
        // Subscription Pose
        geometry_msgs::msg::PoseStamped latest_pose_;
        bool pose_received_ = false;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pos;

        // Path tracking parameters
        double LA_const_;
        double linVel_const_;
        double angVelClamp_;
        double turnRate_;
        double stopDist_;

    public:
        PathTrackingNode() : Node("pathTracking")
        {
            this->declare_parameter<double>("LA_const", 0.5);
            this->declare_parameter<double>("linVel_const", 0.15);
            this->declare_parameter<double>("angVelClamp", 2.0);
            this->declare_parameter<double>("turnRateInPlace", 1.0);
            this->declare_parameter<double>("stopDist", 0.1);

            LA_const_ = this->get_parameter("LA_const").as_double();
            linVel_const_ = this->get_parameter("linVel_const").as_double();
            angVelClamp_ = this->get_parameter("angVelClamp").as_double();
            turnRate_ = this->get_parameter("turnRateInPlace").as_double();
            stopDist_ = this->get_parameter("stopDist").as_double();

            // Subscribe to Pose
            sub_pos = this->create_subscription<geometry_msgs::msg::PoseStamped>("/current_pose", 10, std::bind(&PathTrackingNode::poseCallback, this, _1));

            // Subscribe to waypoints
            sub_wp = this->create_subscription<nav_msgs::msg::Path>("micro_waypoints", 10, std::bind(&PathTrackingNode::pathCallback, this, _1));

            // Publish to cmd_vel
            pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        }

    // Pose callback
    private:
        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            latest_pose_ = *msg;
            pose_received_ = true;
        }

    // pathCallback
    private:
        void pathCallback(const nav_msgs::msg::Path::SharedPtr mpoints) 
        {
            if (!pose_received_) return;  // wait until we have a pose

            if (mpoints->poses.empty()) return;

            // Convert path to  eigen matrix
            Eigen::MatrixXd micropoints(mpoints->poses.size(), 2);
            
            for (size_t i = 0; i < mpoints->poses.size(); ++i)
            {
                micropoints(i,0) = mpoints->poses[i].pose.position.x;
                micropoints(i,1) = mpoints->poses[i].pose.position.y;
            }

            double x = latest_pose_.pose.position.x;
            double y = latest_pose_.pose.position.y;
            double yaw = latest_pose_.pose.position.z;

            int rover_id = static_cast<int>(latest_pose_.pose.orientation.x);

            Eigen::Vector3d state = {x, y, yaw};

            int stopIF = stop_flag(state, micropoints, stopDist_);

            // Call function
            Eigen::Vector2d control = PP_single(state, micropoints, linVel_const_, LA_const_, stopIF, angVelClamp_, turnRate_);

            // Publish command
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = control(0);
            cmd.linear.y = rover_id;
            cmd.angular.z = control(1);
            pub_->publish(cmd);
        }

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_wp;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    std::cout << "pathTracking started running!" << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathTrackingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    std::cout << "pathTracking finished!" << std::endl;
    return 0;
}