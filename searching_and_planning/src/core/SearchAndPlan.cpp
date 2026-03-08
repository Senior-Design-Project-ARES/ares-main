#include "searching_and_planning/core/SearchAndPlan.h"

ares::SearchAndPlanCore::SearchAndPlanCore(const size_t& map_width, const size_t& map_height, const std::pair<double, double>& x, const std::pair<double, double>& y, const std::vector<int8_t>& FE_map, const Target& target, const searching_and_planning::Config& config)
    : config(config),
      map_width(map_width),
      map_height(map_height),
      resolution((x.second - x.first) / map_width),
      origin(Eigen::Vector2d(x.first, y.first)),
      FE_map(FE_map),
      front_expl(static_cast<int>(map_width), static_cast<int>(map_height), resolution, origin, FE_map, config),
      grid_map(static_cast<int>(map_width), static_cast<int>(map_height), x.first, x.second, y.first, y.second, -1),
      target(target)
{
    // Initialize the grid map with the occupancy data
    updateGrid();
}

void ares::SearchAndPlanCore::updateGrid(){
    // This is not really required if I code it properly, but this is more ituitive for now. So people know they are updating the map,
    for(size_t i = 0; i < map_height; i++){
        for(size_t j = 0; j < map_width; j++){
            grid_map(j, i) = FE_map[i*map_width + j];
        }
    }
}

void ares::SearchAndPlanCore::addPathObstacles2Grid(const std::vector<Path2D>& paths){
    int num_other_rover = paths.size();

    if (num_other_rover == 0) {
        return;
    }

    // get all rover current path between frontier and add one additional point
    std::vector<std::vector<Eigen::Vector2d>> rovers_current_path(num_other_rover);
    for(int other_id = 0; other_id < num_other_rover; other_id++){

        if (paths[other_id].waypoints.size() == 0) {
            rovers_current_path[other_id].push_back(paths[other_id].waypoints.back());
            continue;
        }

        rovers_current_path[other_id].push_back(paths[other_id].waypoints[0]);
        for(size_t i = 1; i < paths[other_id].waypoints.size(); i++){
            rovers_current_path[other_id].push_back((paths[other_id].waypoints[i] + paths[other_id].waypoints[i-1]) / 2.0);
            rovers_current_path[other_id].push_back(paths[other_id].waypoints[i]);
        }
    }


    // add other rover's position as obstacles
    for(int other_id = 0; other_id < num_other_rover; other_id++){
        double total_radius = 2*config.rover_radius;
        for(const auto& point : rovers_current_path[other_id]){
            int radius_in_cell = ceil(total_radius*config.radius_inflation /((grid_map.x0Bounds().second - grid_map.x0Bounds().first) / grid_map.size().first));
            for(int dx = -radius_in_cell; dx <= radius_in_cell; dx++){
                for(int dy = -radius_in_cell; dy <= radius_in_cell; dy++){
                    int cell_x, cell_y;
                    std::tie(cell_x, cell_y) = grid_map.getCellFromPoint(point(0), point(1));
                    int new_x = cell_x + dx;
                    int new_y = cell_y + dy;
                    if(new_x < 0 || new_x >= static_cast<int>(grid_map.size().first) || new_y < 0 || new_y >= static_cast<int>(grid_map.size().second)) continue;
                    double dist = sqrt(dx*dx + dy*dy) * ((grid_map.x0Bounds().second - grid_map.x0Bounds().first) / grid_map.size().first);
                    if(dist <= total_radius*config.radius_inflation && grid_map(new_x, new_y) != -1){
                        grid_map(new_x, new_y) = 1;
                    }
                }
            }
        }

    }
}

ares::Path2D ares::SearchAndPlanCore::runSingle(const Eigen::Vector2d current_location, const std::vector<ares::Path2D>& other_rover_paths){
    // initialize path
    ares::Path2D path;
    
    // initialize path planner
    // MyGenericRRT my_rrt(0.05, 7500, 0.3);
    // SST my_sst(0.5, 0.2);
    // MyKinoRRT my_kinorrt(10000, 10);

    // find frontiers
    LOG("Finding frontiers....");
    std::vector<std::pair<Eigen::Vector2d, int>> points = front_expl.run();
    if(points.size() == 0){
        ERROR("no more frontier");
        return path;
    }
    else{
        LOG("Found " << points.size() << " frontier points.");
    }
    
    // determine next point to explore
    Eigen::Vector2d next_point;
    if(target.found){
        next_point = target.position;
    }
    else{
        next_point = nextPoint(points, current_location);
    }

    // plan path to next point
    while(points.size() != 0){
        LOG("Planning path....");
        path = runWithGoal(current_location, next_point, other_rover_paths);

        if(path.valid){
            break;
        }

        next_point = nextPoint(points, current_location);
    }
    if(!path.valid){
        ERROR("cannot find path to any frontier");
        return path;
    }
    LOG("Planning done.");

    // amp::Path2D path_2d;
    // path_2d.waypoints = raw_path.getWaypoints2D();

    // for(int i = 1; i < path_2d.waypoints.size(); i++){
    //     path.waypoints.push_back(path_2d.waypoints[i]);
    //     current_location = path_2d.waypoints[i];
    //     LOG("Updating map....");
    //     C_space.UpdateDiskMapAroundPoint(current_location);
    //     if(!target_found && state(problem.agent_properties[rover_id].q_goal) != -1){
    //         break;
    //     }
    // }



    // current_location = next_point;
    // path.waypoints.insert(path.waypoints.end(), path_2d.waypoints.begin()+1, path_2d.waypoints.end());
    // frontier_path.waypoints.push_back(current_location);

    // path.waypoints = raw_path.getWaypoints2D();
    // frontier_path.waypoints.push_back(path.waypoints.front());
    // frontier_path.waypoints.push_back(path.waypoints.back());


    // rovers_and_frountier_path.agent_paths[0] = path;
    // rovers_and_frountier_path.agent_paths[1] = frontier_path;
    return path;
}

ares::Path2D ares::SearchAndPlanCore::runWithGoal(const Eigen::Vector2d current_location, const Eigen::Vector2d goal_location, const std::vector<Path2D>& other_rover_paths){
    // update gird map and add other rover paths as obstacles
    updateGrid();
    addPathObstacles2Grid(other_rover_paths);

    // initialize path
    ares::Path2D path;
    
    // initialize path planner
    MyGenericRRT my_rrt(0.05, 7500, 0.3);

    //initialize collision checker
    Point2DCollisionCheckerGrid collision_checker(grid_map);

    // plan path to next point
    ares::Path raw_path;
    raw_path.valid = false;

    raw_path = my_rrt.planND(eigen2dToEigenXd(current_location), eigen2dToEigenXd(goal_location), collision_checker);

    path.valid = raw_path.valid;
    path.waypoints = raw_path.getWaypoints2D();

    return path;
}

Eigen::Vector2d ares::SearchAndPlanCore::nextPoint(std::vector<std::pair<Eigen::Vector2d, int>>& point_of_interest, Eigen::Vector2d location){
    double distance = (location - point_of_interest[0].first).norm();
    int nextPoint = 0;
    for(size_t i = 1; i < point_of_interest.size(); i++){
        double temp_distance = (location - point_of_interest[i].first).norm();

        if(distance < config.lidar_radius*1.2 && temp_distance < config.lidar_radius*1.2){
            // Choose the point in the largest frontier region
            if(point_of_interest[i].second > point_of_interest[nextPoint].second){
                nextPoint = i;
                distance = temp_distance;
            }
            continue;
        }
        else if(temp_distance < distance){
            nextPoint = i;
            distance = temp_distance;
        }
    }
    Eigen::Vector2d return_point = point_of_interest[nextPoint].first;
    point_of_interest.erase(point_of_interest.begin() + nextPoint);
    return return_point;
}
