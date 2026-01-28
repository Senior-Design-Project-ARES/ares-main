#include "single_plan_server.h"

// create a path planning node that provides a service to compute paths
PathPlanningServer::PathPlanningServer()
    : Node("path_planning_server"),
      planner(MAP_WIDTH, MAP_HEIGHT, std::make_pair(X_MIN, X_MAX), std::make_pair(Y_MIN, Y_MAX), map, target)
{
    // get parameters
    this->declare_parameter<int32_t>("rover_id", -1);
    rover_id = static_cast<int32_t>(this->get_parameter("rover_id").as_int());


    // create service and say ready
    service_ = this->create_service<cartographer_ros_msgs::srv::TrajectoryQuery>(
        "get_path",
        std::bind(&PathPlanningServer::handle_trajectory_query, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Path planning server ready.");

    // subscribe to current pose topic
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "current_pose",
        10,
        std::bind(&PathPlanningServer::poseCallback, this, std::placeholders::_1));

    // subscribe to other rover paths if needed
    other_rover_paths_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "other_rover_paths",
        10,
        std::bind(&PathPlanningServer::otherRoverPathsCallback, this, std::placeholders::_1));
}

void PathPlanningServer::handle_trajectory_query(
    const std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Request> request,
    std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Response> response){

    RCLCPP_INFO(this->get_logger(), "Received trajectory query for trajectory_id: %d", 
                request->trajectory_id);


    ares::Path2D path;
    if (request->trajectory_id == 0){
        // run planner and consider other rover paths
        path = planner.runSingle(current_location, other_rover_paths_vector);
    }

    if (request->trajectory_id == 1){
        // only consider currnet location of other rovers
        std::vector<ares::Path2D> other_rover_current_locations;
        for (const auto& pair : other_rover_locations) {
            ares::Path2D single_point_path;
            single_point_path.waypoints.push_back(pair.second);
            other_rover_current_locations.push_back(single_point_path);
        }
        path = planner.runSingle(current_location, other_rover_current_locations);
    }

    if (!path.valid) {
        response->status.code = 1;  // Failure
        response->status.message = "Failed to compute path";
        RCLCPP_WARN(this->get_logger(), "Path planning failed.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Path planning succeeded with %zu waypoints.", 
                path.waypoints.size());
    
    response->status.code = 0;  // Success
    response->status.message = "Path computed successfully";

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp = this->now();

    // path output to response
    for (const auto& waypoint : path.waypoints) {
        pose.pose.position.x = waypoint[0];
        pose.pose.position.y = waypoint[1];
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = static_cast<double>(rover_id); // I know I sholud not do this but going to use it for rover id.
        response->trajectory.push_back(pose);
    }
    return;
}

void PathPlanningServer::otherRoverPathsCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    int32_t other_rover_id = static_cast<int32_t>(msg->poses[0].pose.orientation.x);

    ares::Path2D other_path;
    for (const auto& pose_stamped : msg->poses) {
        Eigen::Vector2d waypoint;
        waypoint(0) = pose_stamped.pose.position.x;
        waypoint(1) = pose_stamped.pose.position.y;
        other_path.waypoints.push_back(waypoint);
    }

    Path2D_for_rover path_info;
    path_info.rover_id = other_rover_id;
    path_info.path = other_path;
    path_info.timestamp = msg->header.stamp;

    other_rover_paths[other_rover_id] = path_info;

    // Update the vector of other rover paths for planning
    other_rover_paths_vector.clear();
    for (const auto& pair : other_rover_paths) {
        other_rover_paths_vector.push_back(pair.second.path);
    }

    RCLCPP_INFO(this->get_logger(), "Updated path for rover_id: %d with %zu waypoints",
                other_rover_id, other_path.waypoints.size());
}

void PathPlanningServer::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_location(0) = msg->pose.position.x;
    current_location(1) = msg->pose.position.y;
}

void PathPlanningServer::otherRoverLocationsCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    int32_t other_rover_id = static_cast<int32_t>(msg->pose.orientation.x);
    Eigen::Vector2d location;
    location(0) = msg->pose.position.x;
    location(1) = msg->pose.position.y;

    other_rover_locations[other_rover_id] = location;
}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanningServer>());
  rclcpp::shutdown();
  return 0;
}