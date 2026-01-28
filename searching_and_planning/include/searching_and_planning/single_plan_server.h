#include "rclcpp/rclcpp.hpp"
#include "cartographer_ros_msgs/srv/trajectory_query.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "SearchAndPlan.h"

struct Path2D_for_rover{
    int32_t rover_id;
    ares::Path2D path;
    rclcpp::Time timestamp;
};

// create a path planning node that provides a service to compute paths
class PathPlanningServer : public rclcpp::Node 
{
public:
    PathPlanningServer() : Node("path_planning_server"),
    planner(MAP_WIDTH, MAP_HEIGHT, std::make_pair(X_MIN, X_MAX), std::make_pair(Y_MIN, Y_MAX), map, target){
        service_ = this->create_service<cartographer_ros_msgs::srv::TrajectoryQuery>(
            "get_path",
            std::bind(&PathPlanningServer::handle_trajectory_query, this,
                std::placeholders::_1, std::placeholders::_2));
    
        RCLCPP_INFO(this->get_logger(), "Path planning server ready.");
    }

private:
    void handle_trajectory_query(
        const std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Request> request,
        std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Response> response);

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void otherRoverPathsCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void otherRoverLocationsCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Service<cartographer_ros_msgs::srv::TrajectoryQuery>::SharedPtr service_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr other_rover_paths_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr other_rover_locations_subscription_;

    Eigen::Vector2d current_location;
    std::map<int32_t, Path2D_for_rover> other_rover_paths;
    std::vector<ares::Path2D> other_rover_paths_vector;
    std::map<int32_t, Eigen::Vector2d> other_rover_locations;

    Target target;
    std::vector<int8_t> map = std::vector<int8_t>(MAP_WIDTH * MAP_HEIGHT, 0); // Example empty map
    ares::SearchAndPlanCore planner;
    int32_t rover_id = -1;
};