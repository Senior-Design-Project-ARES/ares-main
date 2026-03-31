#include "searching_and_planning/single_plan_server.h"

// create a path planning node that provides a service to compute paths
PathPlanningServer::PathPlanningServer(const searching_and_planning::Config& config)
    : Node("path_planning_server"),
      current_location(Eigen::Vector2d::Zero()),
      target(),
      config(config),
      map(std::vector<int8_t>(config.map_width * config.map_height, -1)), // Example empty map
      FE_map(std::vector<int8_t>(config.map_width * config.map_height, -1)), // Example empty map
      planner(config.map_width, config.map_height, std::make_pair(config.x_min, config.x_max), std::make_pair(config.y_min, config.y_max), FE_map, target, config)
{
    // get parameters
    this->declare_parameter<int32_t>("rover_id", -1);
    rover_id = static_cast<int32_t>(this->get_parameter("rover_id").as_int());

    this->declare_parameter<bool>("debug", false);
    debug = this->get_parameter("debug").as_bool();

    if (debug) {
        // create publisher for cspace map for debugging
        cspace_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/cspace_map",
            10);
    }

    // subscribe to current pose topic
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/current_pose",
        10,
        std::bind(&PathPlanningServer::poseCallback, this, std::placeholders::_1));

    // subscribe to other rover paths if needed
    rover_paths_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_paths",
        10,
        std::bind(&PathPlanningServer::otherRoverPathsCallback, this, std::placeholders::_1));

    // subscribe to map updates
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        10,
        std::bind(&PathPlanningServer::mapCallback, this, std::placeholders::_1));

    // subscribe to target location
    target_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_location",
        10,
        std::bind(&PathPlanningServer::targetCallback, this, std::placeholders::_1));

    // create service and say ready
    // service_ = this->create_service<cartographer_ros_msgs::srv::TrajectoryQuery>(
    //     "get_path",
    //     std::bind(&PathPlanningServer::handle_trajectory_query, this,
    //               std::placeholders::_1, std::placeholders::_2));
    
    // RCLCPP_INFO(this->get_logger(), "Path planning server ready.");
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
        // only consider current location of other rovers
        std::vector<ares::Path2D> other_rover_current_locations;
        for (const auto& pair : other_rover_locations) {
            ares::Path2D single_point_path;
            single_point_path.waypoints.push_back(pair.second);
            other_rover_current_locations.push_back(single_point_path);
        }
        
        if (!target.found) {
            RCLCPP_WARN(this->get_logger(), "Target not set yet, cannot plan path for trajectory_id 1");
            response->status.code = 1;  // Failure
            response->status.message = "Target not set";
            return;
        }
        
        // path = planner.runSingle(current_location, other_rover_current_locations);
        path = planner.runWithGoal(current_location, target.position, other_rover_current_locations);
    }

    if (!path.valid) {
        response->status.code = 1;  // Failure
        response->status.message = "Failed to compute path";
        RCLCPP_WARN(this->get_logger(), "Path planning failed.");
        
        geometry_msgs::msg::PoseStamped failure_pose;
        failure_pose.header.frame_id = "world";
        failure_pose.header.stamp = this->now();
        failure_pose.pose.position.x = -1.0;
        failure_pose.pose.position.y = -1.0;
        failure_pose.pose.position.z = 0.0;
        failure_pose.pose.orientation.x = static_cast<double>(rover_id);
        response->trajectory.push_back(failure_pose);
        
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Path planning succeeded with %zu waypoints.", 
                path.waypoints.size());


    // compare the last waypoint with the target position, if they are close enough, return code 2
    if ((target.found) && (path.waypoints.back() - target.position).norm() < 0.01) {
        response->status.code = 2;  // Success, target reached
        response->status.message = "Path computed successfully, target reached";
    }
    else{
        response->status.code = 0;  // Success
        response->status.message = "Path computed successfully";
    }

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

    if (other_rover_id == rover_id) {
        // Ignore paths from self
        return;
    }

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
    int32_t id = static_cast<int32_t>(msg->pose.orientation.x);

    // RCLCPP_INFO(this->get_logger(), "Received pose for rover_id: %d", id);

    if (id == rover_id) {
        current_location(0) = msg->pose.position.x;
        current_location(1) = msg->pose.position.y;

        RCLCPP_DEBUG(this->get_logger(), "Updated current location to (%.2f, %.2f)",
                    current_location(0), current_location(1));
        return;
    }
    Eigen::Vector2d location;
    location(0) = msg->pose.position.x;
    location(1) = msg->pose.position.y;
    other_rover_locations[id] = location;

    RCLCPP_DEBUG(this->get_logger(), "Updated location for rover_id: %d to (%.2f, %.2f)",
                id, other_rover_locations[id](0), other_rover_locations[id](1));
}

void PathPlanningServer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Update the internal map representation
    RCLCPP_DEBUG(this->get_logger(), "Received map update with size: %zu", msg->data.size());

    if (msg->data.size() != config.map_width * config.map_height) {
        RCLCPP_ERROR(this->get_logger(), "Received map size does not match expected dimensions.");
        RCLCPP_ERROR(this->get_logger(), "Expected size: %zu, Received size: %zu", config.map_width * config.map_height, msg->data.size());
        return;
    }
    for (size_t i = 0; i < msg->data.size(); ++i) {
        map[i] = msg->data[i];
    }
    updateFEMap();

    if (debug) {
        auto cspace_msg = nav_msgs::msg::OccupancyGrid();
        cspace_msg.header = msg->header;
        cspace_msg.info = msg->info;
        cspace_msg.data = FE_map;
        cspace_map_publisher_->publish(cspace_msg);
    }

    if (!service_) {
        // Now that we have received the first map, we can create the service
        service_ = this->create_service<cartographer_ros_msgs::srv::TrajectoryQuery>(
            "get_path",
            std::bind(&PathPlanningServer::handle_trajectory_query, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Path planning server ready.");
    }
    return;
}

void PathPlanningServer::updateFEMap() {
    // Placeholder for updating the FE map based on the current map and target information
    // This function can be called after receiving a new map or target update
    // For example, you might want to mark cells around the target as more desirable
    // or mark cells around other rovers' paths as less desirable.
    std::vector<std::pair<int, int>> unknow_to_blow_up;
    std::vector<std::pair<int, int>> obstacle_to_blow_up;
    std::vector<int8_t> temp_FEMap = std::vector<int8_t>(config.map_width * config.map_height, -1);
    for (size_t i = 0; i < map.size(); ++i) {
        temp_FEMap[i] = map[i];
    }

    for(size_t i = 0; i < config.map_width; i++){
        for(size_t j = 0; j < config.map_height; j++){
            // find unknow cells that are serrounded by free cells
            if (map[i+j*config.map_width] == -1 && IsSerroundingFree(map, i, j)){
                if (IsSerroundingAllFree(map, i, j)){
                    temp_FEMap[i+j*config.map_width] = 0;
                    continue;
                }
                unknow_to_blow_up.push_back({i, j}); // find unknow cells that are serrounded by free cells
            }
            else if (map[i+j*config.map_width] == 1){
                obstacle_to_blow_up.push_back({i, j}); // find obstacle cells that are serrounded by free cells
            }
        }
    }

    // Blow up unknow cells to size of disk
    for(size_t i = 0; i < unknow_to_blow_up.size(); i++){
        BlowUpPoint(temp_FEMap, unknow_to_blow_up[i].first, unknow_to_blow_up[i].second, -1);
        // int cell_x = unknow_to_blow_up[i].first;
        // int cell_y = unknow_to_blow_up[i].second;
        // for(int dx = - ceil(radius_in_cells); dx <= ceil(radius_in_cells); dx++){
        //     for(int dy = - ceil(radius_in_cells); dy <= ceil(radius_in_cells); dy++){
        //         double distance = sqrt(dx*dx + dy*dy);
        //         if (distance < ceil(radius_in_cells)){
        //             int new_x = cell_x + dx;
        //             int new_y = cell_y + dy;
        //             if(new_x >= 0 && new_x < static_cast<int>(config.map_width) && new_y >= 0 && new_y < static_cast<int>(config.map_height)){
        //                 temp_FEMap[new_x + new_y * config.map_width] = -1;
        //             }
        //         }
        //     }
        // }
    }

    // Blow up obstacle cells to size of disk
    for(size_t i = 0; i < obstacle_to_blow_up.size(); i++){
        BlowUpPoint(temp_FEMap, obstacle_to_blow_up[i].first, obstacle_to_blow_up[i].second, 1);
        // int cell_x = obstacle_to_blow_up[i].first;;
        // int cell_y = obstacle_to_blow_up[i].second;
        // for(int dx = - ceil(radius_in_cells); dx <= ceil(radius_in_cells); dx++){
        //     for(int dy = - ceil(radius_in_cells); dy <= ceil(radius_in_cells); dy++){
        //         double distance = sqrt(dx*dx + dy*dy);
        //         if (distance <= ceil(radius_in_cells)){
        //             int new_x = cell_x + dx;
        //             int new_y = cell_y + dy;
        //             if(new_x >= 0 && new_x < static_cast<int>(config.map_width) && new_y >= 0 && new_y < static_cast<int>(config.map_height)){
        //                 temp_FEMap[new_x + new_y * config.map_width] = 1;
        //             }
        //         }
        //     }
        // }
    }

    for(size_t i = 0; i < FE_map.size(); ++i) {
        FE_map[i] = temp_FEMap[i];
    }
}

void PathPlanningServer::BlowUpPoint(std::vector<int8_t>& _map, int cell_x, int cell_y, int value){
    double radius_in_cells = config.rover_radius * config.radius_inflation / ((config.x_max - config.x_min) / config.map_width);
    for(int dx = - ceil(radius_in_cells); dx <= ceil(radius_in_cells); dx++){
        for(int dy = - ceil(radius_in_cells); dy <= ceil(radius_in_cells); dy++){
            double distance = sqrt(dx*dx + dy*dy);
            if (distance < ceil(radius_in_cells)){
                int new_x = cell_x + dx;
                int new_y = cell_y + dy;
                if(new_x >= 0 && new_x < static_cast<int>(config.map_width) && new_y >= 0 && new_y < static_cast<int>(config.map_height)){
                    _map[new_x + new_y * config.map_width] = value;
                }
            }
        }
    }
}

bool PathPlanningServer::IsSerroundingFree(const std::vector<int8_t>& _map, int i, int j) {
    for(int dx = -1; dx <= 1; dx++){
        for(int dy = -1; dy <= 1; dy++){
            if(dx == 0 && dy == 0) continue; // Skip the center cell
            int new_x = i + dx;
            int new_y = j + dy;
            if(new_x < 0 || new_x >= static_cast<int>(config.map_width) || new_y < 0 || new_y >= static_cast<int>(config.map_height)){
                continue; // Out of bounds
            }
            if(_map[new_x + new_y * config.map_width] == 0){
                return true; // At least one surrounding cell is free
            }
        }
    }
    return false; // All surrounding cells are free
}

bool PathPlanningServer::IsSerroundingAllFree(const std::vector<int8_t>& _map, int i , int j){
    for(int dx = -1; dx <= 1; dx++){
        for(int dy = -1; dy <= 1; dy++){
            if(dx == 0 && dy == 0) continue; // Skip the center cell
            int new_x = i + dx;
            int new_y = j + dy;
            if(new_x < 0 || new_x >= static_cast<int>(config.map_width) || new_y < 0 || new_y >= static_cast<int>(config.map_height)){
                continue; // Out of bounds
            }
            if(_map[new_x + new_y * config.map_width] != 0){
                return false; // At least one surrounding cell is not free
            }
        }
    }
    return true; // All surrounding cells are free
}

void PathPlanningServer::targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    target.found = true;
    target.position[0] = msg->pose.position.x;
    target.position[1] = msg->pose.position.y;
}

// void PathPlanningServer::otherRoverLocationsCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//     int32_t other_rover_id = static_cast<int32_t>(msg->pose.orientation.x);
//     Eigen::Vector2d location;
//     location(0) = msg->pose.position.x;
//     location(1) = msg->pose.position.y;
//     other_rover_locations[other_rover_id] = location;
// }

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  
  // Create configuration from YAML file in config folder relative to current working directory
  std::string config_path = (std::filesystem::current_path() / "config" / "config.yaml").string();
  LOG("Loading config from: " << config_path);
  searching_and_planning::Config config(config_path);
  
  // Create and spin the path planning server
  rclcpp::spin(std::make_shared<PathPlanningServer>(config));
  rclcpp::shutdown();
  return 0;
}