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

double curv_calc(int closest_idx, const Eigen::MatrixXd& micropoints)
{
    int P3 = 0;
    int P2 = 1;
    if (closest_idx + 40 < micropoints.rows())
    {
        P3 = closest_idx + 40;
        P2 = closest_idx + 20;
    }
    else
    {
        return 0.7;
    }

    int P1 = closest_idx;

    double x1 = micropoints(P1,0);
    double x2 = micropoints(P2,0);
    double x3 = micropoints(P3,0);
    double y1 = micropoints(P1,1);
    double y2 = micropoints(P2,1);
    double y3 = micropoints(P3,1);

    double curv = (2*std::abs((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1))) / 
    (std::sqrt((std::pow(x2 - x1, 2)+std::pow(y2 - y1, 2))*(std::pow(x3 - x2, 2)+std::pow(y3 - y2, 2))*(std::pow(x1 - x3, 2)+std::pow(y1 - y3, 2))));

    return curv;
}

double linVel_calc(double curv, double linVel_min, double linVel_max)
{
    if(curv == 0.0)
    {
        return linVel_max;
    }

    double linVel = linVel_max - curv * (linVel_max - linVel_min);

    return linVel;
}

double LA_calc(double curv, double LA_min, double LA_max)
{
    if(curv == 0.0)
    {
        return LA_max;
    }

    double LA = LA_max - curv * (LA_max - LA_min);

    return LA;
}

// Pure Pursuit controller
Eigen::Vector2d PP_single(const Eigen::Vector3d& state, const Eigen::MatrixXd& micropoints, int stopIF, double angVelClamp, double linVel_min, double linVel_max, double LA_min, double LA_max, double turnRate, double trackingAngle_)
{
    if (stopIF == 1)
    {
        Eigen::Vector2d controls;
        controls << 0, 0;
        return controls;

    }
    else
    {
        const Eigen::Vector2d backWheel_pos = {state[0] - cos(state[2]) * 0.15, state[1] - sin(state[2]) * 0.15};

        int closest_idx = findClosestWaypointIndex(backWheel_pos, micropoints);
        double curv = curv_calc(closest_idx, micropoints);

        if(curv>1)
            curv=1;
        
        if(curv<-1)
            curv = -1;

        double linVel = linVel_calc(curv, linVel_min, linVel_max);
        double LA = LA_calc(curv, LA_min, LA_max);
        Eigen::Vector2d target = findPointAtDistance(micropoints, backWheel_pos, LA, closest_idx);
        
        long double dy = target(1) - backWheel_pos(1);
        long double dx = target(0) - backWheel_pos(0);

        double targ = std::atan2(dy, dx); 
        double alpha = targ - state[2];
        alpha = std::atan2(std::sin(alpha), std::cos(alpha));  ;                   

        double kappa = (2 * std::sin(alpha)) / LA;
        double angVel = kappa * linVel;

        if(std::abs(alpha) > (trackingAngle_*(M_PI/180)))
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

        // ------------------ Store Controls in 3x1 Vector ----------------
        Eigen::Vector2d controls;

        controls << linVel, angVel;

        return controls;
    }
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
        double angVelClamp_;
        double linVel_min_;
        double linVel_max_;
        double LA_min_;
        double LA_max_;
        double LA_const_;
        double linVel_const_;
        double turnRate_;
        double stopDist_;
        double trackingAngle_;
        int rover_id_;

    public:
        PathTrackingNode() : Node("pathTracking")
        {
            this->declare_parameter<double>("angVelClamp", 2.0);
            this->declare_parameter<double>("linVel_min", 0.05);
            this->declare_parameter<double>("linVel_max", 2.0);
            this->declare_parameter<double>("LA_min", 0.3);
            this->declare_parameter<double>("LA_max", 1.0);
            this->declare_parameter<double>("LA_const", 0.5);
            this->declare_parameter<double>("linVel_const", 0.15);
            this->declare_parameter<double>("turnRateInPlace", 1.0);
            this->declare_parameter<double>("stopDist", 0.1);
            this->declare_parameter<double>("trackingAngle", 30.0);
            this->declare_parameter<int>("rover_id", -1);


            angVelClamp_ = this->get_parameter("angVelClamp").as_double();
            linVel_min_ = this->get_parameter("linVel_min").as_double();
            linVel_max_ = this->get_parameter("linVel_max").as_double();
            LA_min_ = this->get_parameter("LA_min").as_double();
            LA_max_ = this->get_parameter("LA_max").as_double();
            LA_const_ = this->get_parameter("LA_const").as_double();
            linVel_const_ = this->get_parameter("linVel_const").as_double();
            turnRate_ = this->get_parameter("turnRateInPlace").as_double();
            stopDist_ = this->get_parameter("stopDist").as_double();
            trackingAngle_ = this->get_parameter("trackingAngle").as_double();
            rover_id_ = this->get_parameter("rover_id").as_int();

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
            if(latest_pose_.pose.orientation.x == rover_id_)
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
            Eigen::Vector2d control = PP_single(state, micropoints, stopIF, angVelClamp_, linVel_min_, linVel_max_, LA_min_, LA_max_, turnRate_, trackingAngle_);

            // Publish command
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = control(0);
            cmd.linear.y = rover_id;
            cmd.angular.z = control(1);
            pub_->publish(cmd);

            RCLCPP_INFO(this->get_logger(), "Linear Velocity: %f", control(0));
            RCLCPP_INFO(this->get_logger(), "Angular Velocity: %f", control(1));
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