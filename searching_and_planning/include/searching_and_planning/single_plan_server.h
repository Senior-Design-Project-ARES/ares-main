#pragma once

// import ROS2 libraries
#include "rclcpp/rclcpp.hpp"

// import message and service types
#include "cartographer_ros_msgs/srv/trajectory_query.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// import other necessary libraries
#include "searching_and_planning/core/SearchAndPlan.h"
#include "searching_and_planning/tools/MyPath.h"

struct Path2D_for_rover{
    int32_t rover_id;
    ares::Path2D path;
    rclcpp::Time timestamp;
};

// create a path planning node that provides a service to compute paths
class PathPlanningServer : public rclcpp::Node 
{
public:
    PathPlanningServer();

private:
    void handle_trajectory_query(
        const std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Request> request,
        std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Response> response);

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void otherRoverPathsCallback(const nav_msgs::msg::Path::SharedPtr msg);
    // void otherRoverLocationsCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    rclcpp::Service<cartographer_ros_msgs::srv::TrajectoryQuery>::SharedPtr service_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr other_rover_paths_subscription_;
    // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr other_rover_locations_subscription_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;

    Eigen::Vector2d current_location;
    std::map<int32_t, Path2D_for_rover> other_rover_paths;
    std::vector<ares::Path2D> other_rover_paths_vector;
    std::map<int32_t, Eigen::Vector2d> other_rover_locations;

    Target target;
    std::vector<int8_t> map = std::vector<int8_t>(MAP_WIDTH * MAP_HEIGHT, 0); // Example empty map
    ares::SearchAndPlanCore planner;
    int32_t rover_id = -1;
};