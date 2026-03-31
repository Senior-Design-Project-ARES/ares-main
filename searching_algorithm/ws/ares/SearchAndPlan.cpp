#include "SearchAndPlan.h"
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>

SearchAndPlan::SearchAndPlan(const amp::MultiAgentProblem2D& problem_, const int num_rover_):
problem(problem_), 
num_rover(num_rover_), 
num_cells_x((problem.x_max-problem.x_min)*CELL_PER_METER),
num_cells_y((problem.y_max-problem.y_min)*CELL_PER_METER),
C_space(num_cells_x, num_cells_y, problem, problem_.agent_properties[0].radius),
// map_1d(C_space.getDiskMap1D()),
rovers_paths(num_rover_),
frontier_paths(num_rover_),
active_paths(num_rover_),
front_expl(num_cells_x, num_cells_y, 1.0/CELL_PER_METER, Eigen::Vector2d(problem.x_min, problem.y_min), C_space.getDiskMap1D())
// collision_checker(C_space.getDiskMapptr())
{
    // DEBUG("Initializing SearchAndPlan...");
    for(int i = 0; i < num_rover; i++){
        multi_maps.push_back(new amp::GridCSpace2D_T<int8_t>(num_cells_x, num_cells_y, problem.x_min, problem.x_max, problem.y_min, problem.y_max, -1));
        multi_collision_checker.push_back(new Point2DCollisionCheckerGrid(*multi_maps.back()));
    }
    // DEBUG("SearchAndPlan initialized.");
}

amp::MultiAgentPath2D SearchAndPlan::runSingle(int rover_id){
    amp::MultiAgentPath2D rovers_and_frountier_path(2);
    // amp::Path2D& path = rovers_paths.agent_paths[rover_id];
    // amp::Path2D& frontier_path = frontier_paths.agent_paths[rover_id];
    amp::Path2D path;
    amp::Path2D frontier_path;
    no_frountier_left = false;

    // Eigen::Vector2d current_location = path.waypoints.back();
    Eigen::Vector2d current_location = rovers_paths.agent_paths[rover_id].waypoints.back();
    // path.waypoints.push_back(current_location);
    // frontier_path.waypoints.push_back(current_location);

    MyGenericRRT my_rrt(0.05, 7500, 0.3);
    SST my_sst(0.2, 0.1);
    MyKinoRRT my_kinorrt(10000, 10);

    // amp::Visualizer::makeFigure(problem, rovers_paths);
    updateMultiMap(rover_id);
    std::pair<std::size_t, std::size_t> cell = multi_maps[rover_id]->getCellFromPoint(current_location(0), current_location(1));
    multi_maps[rover_id]->operator()(cell.first, cell.second) = 0;

    LOG("Searching frontier for rover " << rover_id << "....");
    std::vector<std::pair<Eigen::Vector2d, int>> points = front_expl.run();
    if(points.size() == 0){
        ERROR("no more frontier");
        no_frountier_left = true;
        return rovers_and_frountier_path;
    }
    
    Eigen::Vector2d next_point;
    std::vector<std::vector<Eigen::Vector2d>>  temp_active_paths;
    if(state(problem.agent_properties[rover_id].q_goal) != -1){
        next_point = problem.agent_properties[rover_id].q_goal;

        target_found = true;
        for(int i = 0; i < num_rover; i++){
            temp_active_paths.push_back(active_paths.agent_paths[i].waypoints);
            if(i != rover_id){
                active_paths.agent_paths[i].waypoints.clear();
            }
        }
        updateMultiMap(rover_id);
        std::pair<std::size_t, std::size_t> cell = multi_maps[rover_id]->getCellFromPoint(current_location(0), current_location(1));
        multi_maps[rover_id]->operator()(cell.first, cell.second) = 0;
        LOG("target found!");
    }
    else{
        next_point = nextPoint(points, current_location);
    }


    amp::Path raw_path;
    raw_path.valid = false;
    while(points.size() != 0){
        LOG("Planning path....");
        raw_path = my_rrt.planND(eigen2dToEigenXd(current_location), eigen2dToEigenXd(next_point), *multi_collision_checker[rover_id]);
        // raw_path = my_sst.planND(eigen2dToEigenXd(current_location), eigen2dToEigenXd(next_point), C_space.getDiskMapptr());

        // MyFirstOrderUnicycle car_agent = MyFirstOrderUnicycle();
        // amp::KinodynamicProblem2D kino_problem;
        // kino_problem.obstacles = problem.obstacles;
        // kino_problem.agent_type = amp::AgentType::FirstOrderUnicycle;
        // kino_problem.q_init = eigen2dToEigenXd(current_location);
        // kino_problem.q_goal.resize(2);
        // kino_problem.q_goal[0] = std::make_pair(next_point(0)-0.1, next_point(0)+0.1);
        // kino_problem.q_goal[1] = std::make_pair(next_point(1)-0.1, next_point(1)+0.1);
        // kino_problem.q_bounds.resize(2);
        // kino_problem.q_bounds[0] = std::make_pair(problem.x_min, problem.x_max);
        // kino_problem.q_bounds[1] = std::make_pair(problem.y_min, problem.y_max);
        // kino_problem.u_bounds.resize(2);
        // kino_problem.u_bounds[0] = std::make_pair(-1.0, 1.0); // linear velocity
        // kino_problem.u_bounds[1] = std::make_pair(-M_PI/2, M_PI/2); // angular velocity
        // kino_problem.dt_bounds = std::make_pair(0.0, 0.5);
        // kino_problem.agent_dim.length = 0.5;
        // kino_problem.agent_dim.width = 0.3;
        // raw_path = my_kinorrt.plan(kino_problem, car_agent);
        // printf("Raw path valid: %d\n", raw_path.valid);
        // if (target_found && raw_path.valid){
        //     amp::Visualizer::makeFigure(*multi_maps[rover_id]);
        //     amp::Visualizer::makeFigure(problem, rovers_paths);
        // }
        if(raw_path.valid){
            break;
        }
        if (target_found && next_point == problem.agent_properties[rover_id].q_goal){
            for(int i = 0; i < num_rover; i++){
                if(i != rover_id){
                    active_paths.agent_paths[i].waypoints = temp_active_paths[i];
                }
            }
        }
        next_point = nextPoint(points, current_location);
    }
    if(!raw_path.valid){
        ERROR("cannot find path to any frontier");
        no_frountier_left = true;
        return rovers_and_frountier_path;
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

    path.waypoints = raw_path.getWaypoints2D();
    frontier_path.waypoints.push_back(path.waypoints.front());
    frontier_path.waypoints.push_back(path.waypoints.back());


    rovers_and_frountier_path.agent_paths[0] = path;
    rovers_and_frountier_path.agent_paths[1] = frontier_path;
    return rovers_and_frountier_path;
}

Eigen::Vector2d SearchAndPlan::nextPoint(std::vector<std::pair<Eigen::Vector2d, int>>& point_of_interest, Eigen::Vector2d location){
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
    Eigen::Vector2d return_point = point_of_interest[nextPoint].first;
    point_of_interest.erase(point_of_interest.begin() + nextPoint);
    return return_point;
}

Eigen::Vector2d SearchAndPlan::nextPoint_closest(std::vector<std::pair<Eigen::Vector2d, int>>& point_of_interest, Eigen::Vector2d location){
    double distance = (location - point_of_interest[0].first).norm();
    int nextPoint = 0;
    for(int i = 1; i < point_of_interest.size(); i++){
        double temp_distance = (location - point_of_interest[i].first).norm();
        if(temp_distance < distance){
            nextPoint = i;
            distance = temp_distance;
        }
    }
    Eigen::Vector2d return_point = point_of_interest[nextPoint].first;
    point_of_interest.erase(point_of_interest.begin() + nextPoint);
    return return_point;
}

Eigen::Vector2d SearchAndPlan::nextPoint_largestFrontier(std::vector<std::pair<Eigen::Vector2d, int>>& point_of_interest, Eigen::Vector2d location){
    int nextPoint = 0;
    for(int i = 1; i < point_of_interest.size(); i++){
        if(point_of_interest[i].second > point_of_interest[nextPoint].second){
            nextPoint = i;
        }
    }
    Eigen::Vector2d return_point = point_of_interest[nextPoint].first;
    point_of_interest.erase(point_of_interest.begin() + nextPoint);
    return return_point;
}

int SearchAndPlan::state(Eigen::Vector2d location){
    const amp::GridCSpace2D_T<int8_t>& map = getDiskMapptr();
    auto[i, j] = map.getCellFromPoint(location(0), location(1));
    return map(i, j);
}

amp::MultiAgentPath2D SearchAndPlan::run(){
    C_space.reset();
    target_found = false;

    for (int rover_id = 0; rover_id < num_rover; rover_id++){
        rovers_paths.agent_paths[rover_id].waypoints.clear();
        rovers_paths.agent_paths[rover_id].waypoints.push_back(problem.agent_properties[rover_id].q_init);
        
        rovers_paths.agent_paths[rover_id].valid = false;
        frontier_paths.agent_paths[rover_id].waypoints.clear();
        frontier_paths.agent_paths[rover_id].waypoints.push_back(problem.agent_properties[rover_id].q_init);
    }

    for(int i = 0; i < num_rover; i++){
        C_space.UpdateDiskMapAroundPoint(problem.agent_properties[i].q_init);
    }
    // DEBUG("Initial map constructed.");

    std::vector<int> current_waypoint_indexs(num_rover, 0);
    bool target_reached = false;
    while(!no_frountier_left && !target_reached){
        for (int rover_id = 0; rover_id < num_rover && !no_frountier_left && !target_reached; rover_id++){
            if(target_found){
                if(active_paths.agent_paths[rover_id].waypoints.size() == 0){
                    continue;
                }
                if((active_paths.agent_paths[rover_id].waypoints[current_waypoint_indexs[rover_id]] - problem.agent_properties[rover_id].q_goal).norm() < 0.1){
                    frontier_paths.agent_paths[rover_id].waypoints.push_back(problem.agent_properties[rover_id].q_goal);
                    target_reached = true;
                }
            }


            if(current_waypoint_indexs[rover_id] < active_paths.agent_paths[rover_id].waypoints.size()){
                Eigen::Vector2d current_location = active_paths.agent_paths[rover_id].waypoints[current_waypoint_indexs[rover_id]];
                LOG("updating map: " << current_location.transpose());
                C_space.UpdateDiskMapAroundPoint(current_location);
                rovers_paths.agent_paths[rover_id].waypoints.push_back(current_location);
                
                if(current_waypoint_indexs[rover_id] == 0){
                    frontier_paths.agent_paths[rover_id].waypoints.push_back(current_location);
                }
                else if(!target_found && state(problem.agent_properties[rover_id].q_goal) != -1){
                    frontier_paths.agent_paths[rover_id].waypoints.push_back(current_location);
                    active_paths.agent_paths[rover_id].waypoints.clear();
                    current_waypoint_indexs[rover_id] = 0;
                }
                
                current_waypoint_indexs[rover_id]++;
                continue;
            }


            // if (active_paths.agent_paths[rover_id].waypoints.size() > 0){
            //     Eigen::Vector2d current_location = active_paths.agent_paths[rover_id].waypoints.back();
            //     C_space.UpdateDiskMapAroundPoint(current_location);
            //     rovers_paths.agent_paths[rover_id].waypoints.push_back(current_location);
                
            //     if(active_paths.agent_paths[rover_id].waypoints.size() == 1){
            //         frontier_paths.agent_paths[rover_id].waypoints.push_back(current_location);
            //     }
            //     else if(!target_found && state(problem.agent_properties[rover_id].q_goal) != -1){
            //         frontier_paths.agent_paths[rover_id].waypoints.push_back(current_location);
            //         active_paths.agent_paths[rover_id].waypoints.clear();
            //     }
                
            //     active_paths.agent_paths[rover_id].waypoints.pop_back();
            //     continue;
            // }


            amp::MultiAgentPath2D single_rover_path = runSingle(rover_id);
            
            // Export disk map to CSV for rover 1
            if (rover_id == 0) {
                static int iteration_counter = 0;
                writeDiskMapToCSV(rover_id, iteration_counter++);
            }
            
            active_paths.agent_paths[rover_id] = single_rover_path.agent_paths[0];
            current_waypoint_indexs[rover_id] = 0;

            // amp::Visualizer::makeFigure(problem, active_paths);
            // amp::Visualizer::makeFigure(getMapptr());
            // single_rover_path.agent_paths[1];
            // break;
        }
        amp::MultiAgentPath2D current_location(num_rover);
        for(int rover_id = 0; rover_id < num_rover; rover_id++){
            current_location.agent_paths[rover_id].waypoints.push_back(rovers_paths.agent_paths[rover_id].waypoints.back());
            // break;
        }
        // break;
        // amp::Visualizer::makeFigure(C_space.getMapptr(), current_location);
        
        // amp::Visualizer::makeFigure(C_space.getMapptr());
        // amp::Visualizer::makeFigure(C_space.getDiskMapptr());
        // amp::Visualizer::makeFigure(problem, rovers_paths);
        // amp::Visualizer::makeFigure(problem, frontier_paths);
        // amp::Visualizer::saveFigures(true, "ares");
    }
    // amp::Visualizer::saveFigures(false, "DR2");

    amp::MultiAgentPath2D rovers_and_frountier_path(num_rover*2);
    for(int rover_id = 0; rover_id < num_rover; rover_id++){
        rovers_and_frountier_path.agent_paths[rover_id] = rovers_paths.agent_paths[rover_id];
        rovers_and_frountier_path.agent_paths[rover_id + num_rover] = frontier_paths.agent_paths[rover_id];
    }
    return rovers_and_frountier_path;
}

std::pair<amp::MultiAgentPath2D, std::vector<double>> SearchAndPlan::runWithTime(){
    
    C_space.reset();
    target_found = false;
    std::vector<double> time_takens;

    for (int rover_id = 0; rover_id < num_rover; rover_id++){
        rovers_paths.agent_paths[rover_id].waypoints.clear();
        rovers_paths.agent_paths[rover_id].waypoints.push_back(problem.agent_properties[rover_id].q_init);
        
        rovers_paths.agent_paths[rover_id].valid = false;
        frontier_paths.agent_paths[rover_id].waypoints.clear();
        frontier_paths.agent_paths[rover_id].waypoints.push_back(problem.agent_properties[rover_id].q_init);
    }

    for(int i = 0; i < num_rover; i++){
        C_space.UpdateDiskMapAroundPoint(problem.agent_properties[i].q_init);
    }
    // DEBUG("Initial map constructed.");

    std::vector<int> current_waypoint_indexs(num_rover, 0);
    bool target_reached = false;
    int no_frountier_count = 0;
    while(!(no_frountier_count >= 3) && !target_reached){
        no_frountier_count = 0;
        for (int rover_id = 0; rover_id < num_rover && !target_reached; rover_id++){
            if(target_found){
                bool need_to_continue = false;
                for (int i = 0; i < num_rover; i++){
                    if(active_paths.agent_paths[i].waypoints.size() == 0){
                        continue;
                    }
                    if((active_paths.agent_paths[i].waypoints.back() - problem.agent_properties[i].q_goal).norm() < 1e-3){
                        need_to_continue = true;
                        break;
                    }
                }
                if(need_to_continue && active_paths.agent_paths[rover_id].waypoints.size() == 0){
                    continue;
                }

                if(active_paths.agent_paths[rover_id].waypoints.size() == 0){
                }
                else if((active_paths.agent_paths[rover_id].waypoints[current_waypoint_indexs[rover_id]] - problem.agent_properties[rover_id].q_goal).norm() < 1e-3){
                    frontier_paths.agent_paths[rover_id].waypoints.push_back(problem.agent_properties[rover_id].q_goal);
                    target_reached = true;
                }
            }

            if(current_waypoint_indexs[rover_id] < active_paths.agent_paths[rover_id].waypoints.size()){
                Eigen::Vector2d current_location = active_paths.agent_paths[rover_id].waypoints[current_waypoint_indexs[rover_id]];
                LOG("updating map: " << current_location.transpose());
                C_space.UpdateDiskMapAroundPoint(current_location);
                rovers_paths.agent_paths[rover_id].waypoints.push_back(current_location);
                
                if(current_waypoint_indexs[rover_id] == 0){
                    frontier_paths.agent_paths[rover_id].waypoints.push_back(current_location);
                }
                // else if(!target_found && state(problem.agent_properties[rover_id].q_goal) != -1){
                //     frontier_paths.agent_paths[rover_id].waypoints.push_back(current_location);
                //     active_paths.agent_paths[rover_id].waypoints.clear();
                //     current_waypoint_indexs[rover_id] = 0;
                // }
                
                current_waypoint_indexs[rover_id]++;
                continue;
            }

            // auto start = std::chrono::high_resolution_clock::now();
            amp::MultiAgentPath2D single_rover_path = runSingle(rover_id);
            // auto end =  std::chrono::high_resolution_clock::now();
            // double time_taken = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            // time_takens.push_back(time_taken);
            if(no_frountier_left){
                no_frountier_count++;
                continue;
            }

            active_paths.agent_paths[rover_id] = single_rover_path.agent_paths[0];
            current_waypoint_indexs[rover_id] = 0;
        }
        amp::MultiAgentPath2D current_location(num_rover);
        for(int rover_id = 0; rover_id < num_rover; rover_id++){
            current_location.agent_paths[rover_id].waypoints.push_back(rovers_paths.agent_paths[rover_id].waypoints.back());
        }
    }

    amp::MultiAgentPath2D rovers_and_frountier_path(num_rover*2);
    for(int rover_id = 0; rover_id < num_rover; rover_id++){
        rovers_and_frountier_path.agent_paths[rover_id] = rovers_paths.agent_paths[rover_id];
        rovers_and_frountier_path.agent_paths[rover_id + num_rover] = frontier_paths.agent_paths[rover_id];
    }
    return {rovers_and_frountier_path, time_takens};
}

void SearchAndPlan::updateMultiMap(int rover_id){
    amp::GridCSpace2D_T<int8_t>* map_ptr = multi_maps[rover_id];
    const amp::GridCSpace2D_T<int8_t>& disk_map = C_space.getDiskMapptr();

    // amp::Visualizer::makeFigure(disk_map);

    double radius = problem.agent_properties[rover_id].radius;

    for(int i = 0; i < num_cells_x; i++){
        for(int j = 0; j < num_cells_y; j++){
            (*map_ptr)(i, j) = disk_map(i, j);
        }
    }


    // get all rover current path between frontier and add one additional point
    std::vector<std::vector<Eigen::Vector2d>> rovers_current_path(num_rover);
    for(int id = 0; id < num_rover; id++){
        if (id == rover_id) continue;

        if (active_paths.agent_paths[id].waypoints.size() == 0) {
            rovers_current_path[id].push_back(rovers_paths.agent_paths[id].waypoints.back());
            continue;
        }
        
        rovers_current_path[id].push_back(active_paths.agent_paths[id].waypoints[0]);
        for(int i = 1; i < active_paths.agent_paths[id].waypoints.size(); i++){
            rovers_current_path[id].push_back((active_paths.agent_paths[id].waypoints[i] + active_paths.agent_paths[id].waypoints[i-1]) / 2.0);
            rovers_current_path[id].push_back(active_paths.agent_paths[id].waypoints[i]);
        }
    }



    // add other rover's position as obstacles
    for(int other_id = 0; other_id < num_rover; other_id++){
        if(other_id == rover_id) continue;
        double total_radius = radius + problem.agent_properties[other_id].radius;
        for(const auto& point : rovers_current_path[other_id]){
            int radius_in_cell = ceil(total_radius*1.5 /((problem.x_max - problem.x_min) / num_cells_x));
            for(int dx = -radius_in_cell; dx <= radius_in_cell; dx++){
                for(int dy = -radius_in_cell; dy <= radius_in_cell; dy++){
                    int cell_x, cell_y;
                    std::tie(cell_x, cell_y) = map_ptr->getCellFromPoint(point(0), point(1));
                    int new_x = cell_x + dx;
                    int new_y = cell_y + dy;
                    if(new_x < 0 || new_x >= num_cells_x || new_y < 0 || new_y >= num_cells_y) continue;
                    double dist = sqrt(dx*dx + dy*dy) * ((problem.x_max - problem.x_min) / num_cells_x);
                    if(dist <= total_radius*1.5 && (*map_ptr)(new_x, new_y) != -1){
                        (*map_ptr)(new_x, new_y) = 1;
                    }
                }
            }
        }

    }
    // amp::Visualizer::makeFigure(*map_ptr);
    // amp::Visualizer::saveFigures(true, "ares");
}

void SearchAndPlan::writeDiskMapToCSV(int rover_id, int iteration) {
    const amp::GridCSpace2D_T<int8_t>& disk_map = C_space.getDiskMapptr();
    auto [num_cells_x, num_cells_y] = disk_map.size();
    
    // Generate filename with rover_id and iteration number
    std::string filename = "rover_" + std::to_string(rover_id) + "_iter_" + std::to_string(iteration) + "_diskmap.csv";
    
    std::ofstream csv_file(filename);
    if (!csv_file.is_open()) {
        ERROR("Failed to open file: " << filename);
        return;
    }
    
    // Write CSV data: each row represents a row in the grid (y coordinate)
    // each column represents a column in the grid (x coordinate)
    for (std::size_t j = 0; j < num_cells_y; j++) {
        for (std::size_t i = 0; i < num_cells_x; i++) {
            csv_file << static_cast<int>(disk_map(i, j));
            if (i < num_cells_x - 1) {
                csv_file << ",";
            }
        }
        csv_file << "\n";
    }
    
    csv_file.close();
    LOG("Exported disk map to " << filename);
}
