#include "SearchAndPlan.h"

SearchAndPlan::SearchAndPlan(const amp::Problem2D& problem_, const int num_rover_):
problem(problem_), 
num_rover(num_rover_), 
num_cells_x((problem.x_max-problem.x_min)*CELL_PER_METER),
num_cells_y((problem.y_max-problem.y_min)*CELL_PER_METER),
lidar_space(num_cells_x, num_cells_y, problem){
    MyLidarEmulateConstructor lidar_space(num_cells_x, num_cells_y, problem);
}

amp::MultiAgentPath2D SearchAndPlan::run(){
    lidar_space.reset();
    bool target_found = false;
    amp::MultiAgentPath2D rovers_path(num_rover);
    amp::MultiAgentPath2D frontier_path(num_rover);

    Eigen::Vector2d current_location = problem.q_init;
    LOG("Creating map....");
    const std::vector<int8_t>& map = lidar_space.construct4point1D(current_location);
    FrontExpl front_expl(num_cells_x, num_cells_y, 1.0/CELL_PER_METER, Eigen::Vector2d(problem.x_min, problem.y_min), map);
    Point2DCollisionCheckerGrid collision_checker(lidar_space.getMapptr());

    MyGenericRRT my_rrt(0.05, 7500, 0.3);

    rovers_path.agent_paths[0].waypoints.push_back(current_location);
    frontier_path.agent_paths[0].waypoints.push_back(current_location);
    while(!target_found){
        // LOG("Updating map....");
        // lidar_space.construct4point1D(current_location);
        amp::Visualizer::makeFigure(lidar_space.getMapptr());
        LOG("Searching frontier....");
        std::vector<std::pair<Eigen::Vector2d, int>> points = front_expl.run();
        if(points.size() == 0){
            ERROR("no more frontier");
            break;
        }
        
        LOG("Planning path....");
        if(state(problem.q_goal) != -1){
            amp::Path raw_path = my_rrt.planND(eigen2dToEigenXd(current_location), eigen2dToEigenXd(problem.q_goal), collision_checker);
            amp::Path2D path_2d;
            path_2d.waypoints = raw_path.getWaypoints2D();
            rovers_path.agent_paths[0].waypoints.insert(rovers_path.agent_paths[0].waypoints.end(), path_2d.waypoints.begin()+1, path_2d.waypoints.end());
            target_found = true;
            LOG("target found!");
            break;
        }
        Eigen::Vector2d next_point = nextPoint(points, current_location);
        amp::Path raw_path = my_rrt.planND(eigen2dToEigenXd(current_location), eigen2dToEigenXd(next_point), collision_checker);
        amp::Path2D path_2d;
        path_2d.waypoints = raw_path.getWaypoints2D();

        for(int i = 1; i < path_2d.waypoints.size(); i++){
            rovers_path.agent_paths[0].waypoints.push_back(path_2d.waypoints[i]);
            current_location = path_2d.waypoints[i];
            LOG("Updating map....");
            lidar_space.construct4point1D(current_location);
            if(state(problem.q_goal) != -1){
                break;
            }

        }

        // current_location = next_point;
        // rovers_path.agent_paths[0].waypoints.insert(rovers_path.agent_paths[0].waypoints.end(), path_2d.waypoints.begin()+1, path_2d.waypoints.end());
        frontier_path.agent_paths[0].waypoints.push_back(current_location);
    }
    rovers_path.agent_paths.push_back(frontier_path.agent_paths[0]);
    return rovers_path;
}

Eigen::Vector2d SearchAndPlan::nextPoint(std::vector<std::pair<Eigen::Vector2d, int>> point_of_interest, Eigen::Vector2d location){
    double distance = (location - point_of_interest[0].first).norm();
    int nextPoint = 0;
    for(int i = 1; i < point_of_interest.size(); i++){
        double temp_distance = (location - point_of_interest[i].first).norm();

        if(distance < LIDARRADIUS*1.2 && temp_distance < LIDARRADIUS*1.2){
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
    return point_of_interest[nextPoint].first;
}

int SearchAndPlan::state(Eigen::Vector2d location){
    const amp::GridCSpace2D_T<int8_t>& map = getMapptr();
    auto[i, j] = map.getCellFromPoint(location(0), location(1));
    return map(i, j);
}
