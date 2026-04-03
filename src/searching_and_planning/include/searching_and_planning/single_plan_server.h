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
#include <filesystem>
#include "searching_and_planning/tools/Logging.h"
#include "searching_and_planning/core/Config.h"

struct Path2D_for_rover{
    int32_t rover_id;
    ares::Path2D path;
    rclcpp::Time timestamp;
};

// create a path planning node that provides a service to compute paths
class PathPlanningServer : public rclcpp::Node 
{
public:
    PathPlanningServer(const searching_and_planning::Config& config);

private:
    void handle_trajectory_query(
        const std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Request> request,
        std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Response> response);

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void otherRoverPathsCallback(const nav_msgs::msg::Path::SharedPtr msg);
    // void otherRoverLocationsCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    void updateFEMap();
    bool IsSerroundingFree(const std::vector<int8_t>& _map, int i, int j);
    bool IsSerroundingAllFree(const std::vector<int8_t>& _map, int i, int j);
    void BlowUpPoint(std::vector<int8_t>& _map, int cell_x, int cell_y, int value);

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cspace_map_publisher_;

    rclcpp::Service<cartographer_ros_msgs::srv::TrajectoryQuery>::SharedPtr service_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rover_paths_subscription_;
    // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr other_rover_locations_subscription_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_subscription_;

    Eigen::Vector2d current_location;
    std::map<int32_t, Path2D_for_rover> other_rover_paths;
    std::vector<ares::Path2D> other_rover_paths_vector;
    std::map<int32_t, Eigen::Vector2d> other_rover_locations;

    Target target;
    const searching_and_planning::Config& config;
    std::vector<int8_t> map;
    std::vector<int8_t> FE_map;
    ares::SearchAndPlanCore planner;
    int32_t rover_id = -1;
    bool debug = false;
    bool map_received = false;
};