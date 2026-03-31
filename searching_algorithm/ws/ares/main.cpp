#include "AMPCore.h"

#include "SearchAndPlan.h"
#include "tools/Environment.h"
#include "hw/HW2.h"
#include "hw/HW8.h"

// #define CELL_PER_METER         // in cell per meter

int main(int argc, char** argv){
    amp::RNG::seed(amp::RNG::randiUnbounded());
    std::srand(static_cast<unsigned int>(std::time(0)));

    // generate enviroument
    amp::Random2DEnvironmentSpecification spec;
    amp::RandomCircularAgentsSpecification ma_spec;
    spec.x_max = 22.0;
    spec.y_max = 9.0;
    ma_spec.n_agents = 1;
    ma_spec.max_agent_radius = 0.15;
    ma_spec.min_agent_radius = 0.15;
    spec.n_obstacles = 60*1.5;
    spec.max_obstacle_region_radius = 0.8;
    spec.path_clearance = 0.3*1.2;



    
    amp::Problem2D problem;
    amp::MultiAgentProblem2D multi_problem;
    amp::CircularAgentProperties agent;

    amp::Deserializer dszr("../../in/problem.yaml");
    problem.deserialize(dszr);
    
    // amp::Visualizer::makeFigure(problem);
    // amp::Visualizer::saveFigures(true, "ares");

    // return 0;

    if(true){
        // amp::Random2DEnvironmentSpecification ori_spec;
        problem = amp::EnvironmentTools::generateRandomPointAgentProblem(spec, 55);

        multi_problem.x_max = spec.x_max;
        multi_problem.y_max = spec.y_max;
        multi_problem.obstacles = problem.obstacles;
        agent.radius = 0.15;
        agent.q_init = problem.q_init;
        agent.q_goal = problem.q_goal;

        multi_problem.agent_properties.push_back(agent);
    }
    
    if(false){
        multi_problem.x_max = problem.x_max;
        multi_problem.y_max = problem.y_max;
        multi_problem.obstacles = problem.obstacles;

        agent.radius = 0.15;
        agent.q_init = problem.q_init;
        agent.q_goal = problem.q_goal;

        multi_problem.agent_properties.push_back(agent);
        multi_problem.agent_properties.push_back(agent);
        multi_problem.agent_properties.push_back(agent);
        multi_problem.agent_properties[1].q_init = Eigen::Vector2d(0.5, 8);
        multi_problem.agent_properties[2].q_init = Eigen::Vector2d(0.7, 5.5);
    }

    std::vector<int8_t> map(12*12, -1);

    if (true){
        for(int i = 0; i < 7; i++){
            for(int j = 0; j < 5; j++){
                map[j*12 +i] = 0;
            }
        }
    }

    if (true){
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 11; j++){
                map[j*12 +i] = 0;
            }
        }
    }

    if (true){
        for (int i = 6; i < 8; i++){
            for (int j = 4; j < 6; j++){
                map[j*12 + i] = 1;
            }
        }
    }

    float resolution = 0.25;

    FrontExpl front_expl(12, 12, resolution, Eigen::Vector2d(0.0, 0.0), map);

    std::vector<std::pair<Eigen::Vector2d, int>> front = front_expl.run();

    // INFO(front[0].first.transpose());
    // INFO(front.size());


    // resolution = 1;
    amp::GridCSpace2D_T<int8_t> grid_map(12, 12, 0, 12, 0, 12, -1);

    DEBUG(map.size());
    for(int i = 0; i < map.size(); i++){
        // DEBUG(i);
        grid_map(i%12, floor(i/12)) = map[i];
    }
    amp::Visualizer::makeFigure(grid_map);

    for(auto& [point, value] : front){
        grid_map(point[0]/resolution, point[1]/resolution) = 2;
    }

    amp::Visualizer::makeFigure(grid_map);
    amp::Visualizer::saveFigures(true, "AIAA");

    // // enviroument for DR2
    // if (false){
    //     problem =  amp::EnvironmentTools::generateRandomPointAgentProblem(spec, 34);

    //     multi_problem.x_max = spec.x_max;
    //     multi_problem.y_max = spec.y_max;
    //     multi_problem.obstacles = problem.obstacles;

    //     agent.radius = 0.15;
    //     agent.q_init = problem.q_init;
    //     agent.q_goal = problem.q_goal;

    //     multi_problem.agent_properties.push_back(agent);
    //     multi_problem.agent_properties.push_back(agent);
    //     multi_problem.agent_properties.push_back(agent);
    //     multi_problem.agent_properties[0].q_init = Eigen::Vector2d(19.11, 8.22);
    //     multi_problem.agent_properties[1].q_init = Eigen::Vector2d(12.81, 8.51);
    //     multi_problem.agent_properties[2].q_init = Eigen::Vector2d(1.11, 8.0);
    // }

    // // show singer rover
    // SearchAndPlan search_plan_algo(multi_problem, multi_problem.numAgents());
    // amp::MultiAgentPath2D rovers_path = search_plan_algo.run();
    // amp::MultiAgentPath2D frontier_path;
    // for(int i = 0; i < multi_problem.numAgents(); i++){
    //     frontier_path.agent_paths.insert(frontier_path.agent_paths.begin(), rovers_path.agent_paths.back());
    //     rovers_path.agent_paths.pop_back();
    // }


    // amp::Visualizer::makeFigure(search_plan_algo.getMapptr());
    // amp::Visualizer::makeFigure(search_plan_algo.getDiskMapptr());
    // amp::Visualizer::makeFigure(multi_problem, rovers_path);
    // amp::Visualizer::makeFigure(multi_problem, frontier_path);
    // amp::Visualizer::saveFigures(true, "ARES");

    // benchmarking for single rover
    // int successful_runs = 0;
    // std::vector<double> run_times;
    // for (int i = 0; i < 100; i ++){
    //     problem = amp::EnvironmentTools::generateRandomPointAgentProblem(spec, 50+i);
    //     multi_problem.obstacles = problem.obstacles;
    //     multi_problem.agent_properties[0].q_init = problem.q_init;
    //     multi_problem.agent_properties[0].q_goal = problem.q_goal;

    //     SearchAndPlan search_plan_algo(multi_problem, 1);
    //     std::pair<amp::MultiAgentPath2D, std::vector<double>> result = search_plan_algo.runWithTime();
    //     amp::MultiAgentPath2D rovers_path = result.first;
    //     rovers_path.agent_paths.pop_back(); // remove frontier path
    //     run_times.insert(run_times.end(), result.second.begin(), result.second.end());

    //     // bool success = amp::HW2::check(rovers_path.agent_paths[0], problem);
    //     bool success = amp::HW8::check(rovers_path, multi_problem);
    //     if(success){
    //         successful_runs += 1;
    //     }
    // }
    // std::list<std::vector<double>> all_run_data;
    // all_run_data.push_back(run_times);
    // std::vector<std::string> labels = {"Single Rover Exploration"};
    // std::string title = "Single Rover Exploration Frontier search and path planning time";
    // std::string xlabel = "";
    // std::string ylabel = "Time (ms)";
    // amp::Visualizer::makeBoxPlot(all_run_data, labels, title, xlabel , ylabel);
    // amp::Visualizer::makeBarGraph({(double)successful_runs}, {"Single Rover Exploration"}, "Number of Successful Runs over 100 runs", "", "Number of Successful Runs");
    // amp::Visualizer::saveFigures(true, "ARES_single_rover_benchmark");
    
    // benchmarking for multi rover
    // int successful_runs = 0;
    // std::vector<double> run_times;
    // for (int i = 0; i < 100; i ++){
    //     multi_problem = amp::EnvironmentTools::generateRandomMultiAgentProblem(spec, ma_spec, 50+i);
    //     std::vector<double> min_distances;
    //     int goal_with_max_min_distance = 0;
    //     for(int j = 0; j < multi_problem.numAgents(); j++){
    //         min_distances.push_back((std::numeric_limits<double>::max)());
    //         double dist = 0.0;
    //         for(int k = 0; k < multi_problem.numAgents(); k++){
    //             dist = (multi_problem.agent_properties[k].q_init - multi_problem.agent_properties[j].q_goal).norm();
    //             if (dist < min_distances[j]){
    //                 min_distances[j] = dist;
    //             }
    //         }
    //     }

    //     for(int j = 0; j < multi_problem.numAgents(); j++){
    //         if (min_distances[j] > min_distances[goal_with_max_min_distance]){
    //             goal_with_max_min_distance = j;
    //         }
    //     }


    //     Eigen::Vector2d common_goal = multi_problem.agent_properties[goal_with_max_min_distance].q_goal;
    //     // Eigen::Vector2d common_goal = Eigen::Vector2d(3.0, 8.0);
    //     // multi_problem.agent_properties[goal_with_max_min_distance].q_goal = common_goal;
    //     for(int j = 0; j < multi_problem.numAgents(); j++){
    //         if (j != goal_with_max_min_distance) multi_problem.agent_properties[j].q_goal = multi_problem.agent_properties[goal_with_max_min_distance].q_goal;
    //     }

    //     SearchAndPlan search_plan_algo(multi_problem, ma_spec.n_agents);
    //     std::pair<amp::MultiAgentPath2D, std::vector<double>> result = search_plan_algo.runWithTime();
    //     amp::MultiAgentPath2D rovers_path = result.first;

    //     for(int j = 0; j < multi_problem.numAgents(); j++){
    //         if ((rovers_path.agent_paths[j].waypoints.back() - multi_problem.agent_properties[j].q_goal).norm() > 1e-3){
    //             multi_problem.agent_properties[j].q_goal = rovers_path.agent_paths[j].waypoints.back();
    //         }
    //         rovers_path.agent_paths.pop_back();
    //     }
    //     // run_times.insert(run_times.end(), result.second.begin(), result.second.end());

    //     // bool success = amp::HW2::check(rovers_path.agent_paths[0], problem);

    //     bool success = amp::HW8::check(rovers_path, multi_problem);
    //     if(success){
    //         successful_runs += 1;
    //     }

    //     for(int j = 0; j < multi_problem.numAgents(); j++){
    //         multi_problem.agent_properties[j].q_goal = common_goal;
    //     }

    //     // amp::Visualizer::makeFigure(multi_problem, rovers_path);
    // }
    // amp::Visualizer::saveFigures(true, "ARES_multi_rover_benchmark");

    // std::list<std::vector<double>> all_run_data;
    // all_run_data.push_back(run_times);
    // std::vector<std::string> labels = {"Single Rover Exploration"};
    // std::string title = "Single Rover Exploration Frontier search and path planning time";
    // std::string xlabel = "";
    // std::string ylabel = "Time (ms)";
    // amp::Visualizer::makeBoxPlot(all_run_data, labels, title, xlabel , ylabel);
    // double successful_runs = 95; // hard coded for multi rover benchmark
    // amp::Visualizer::makeBarGraph({(double)successful_runs}, {"Multi Rover Exploration"}, "Number of Successful Runs over 100 runs", "", "Number of Successful Runs");
    // amp::Visualizer::saveFigures(true, "ARES_single_rover_benchmark");

    // int cell_per_meter = 20;
    // MyDiskAgentCS cspace((multi_problem.x_max-multi_problem.x_min)*cell_per_meter, (multi_problem.y_max-multi_problem.y_min)*cell_per_meter, multi_problem, agent.radius);
    // cspace.UpdateDiskMapAroundPoint(multi_problem.agent_properties[0].q_init);
    // FrontExpl frontier_explore(cspace.cells_x(), cspace.cells_y(), 1.0/cell_per_meter, Eigen::Vector2d(multi_problem.x_min, multi_problem.y_min), cspace.getDiskMap1D());
    // std::vector<std::pair<Eigen::Vector2d, int>> points = frontier_explore.run();
    // cspace.addFrontierToCS(frontier_explore.getCentroidsGrid());

    // amp::Visualizer::makeFigure(multi_problem);
    // // amp::Visualizer::makeFigure(cspace.getMapptr());
    // // amp::Visualizer::makeFigure(cspace.getDiskMapptr());
    // amp::Visualizer::saveFigures(true, "ARES_multi_agent_problem");


    // multi_problem = amp::EnvironmentTools::generateRandomMultiAgentProblem(spec, ma_spec, 1000);
    // for(int i = 1; i < multi_problem.numAgents(); i++){
    //     multi_problem.agent_properties[i].q_goal = multi_problem.agent_properties[0].q_goal;
    // }
    // // DEBUG("Starting ARES planning....");
    // SearchAndPlan search_plan_algo(multi_problem, multi_problem.numAgents());
    // amp::MultiAgentPath2D rovers_path = search_plan_algo.run();
    // amp::MultiAgentPath2D frontier_path;
    // for(int i = 0; i < multi_problem.numAgents(); i++){
    //     frontier_path.agent_paths.insert(frontier_path.agent_paths.begin(), rovers_path.agent_paths.back());
    //     rovers_path.agent_paths.pop_back();
    // }


    // amp::Visualizer::makeFigure(search_plan_algo.getMapptr());
    // // amp::Visualizer::makeFigure(search_plan_algo.getDiskMapptr());
    // amp::Visualizer::makeFigure(multi_problem, rovers_path);
    // amp::Visualizer::makeFigure(multi_problem, frontier_path);
    // amp::Visualizer::saveFigures(true, "ARES");

    

    // int successful_runs = 0;
    // std::vector<double> run_times;
    // std::vector<double> all_successful_runs;
    // std::list<std::vector<double>> all_run_data;
    // for(int i = 0; i < 100; i ++){
    //     multi_problem = amp::EnvironmentTools::generateRandomMultiAgentProblem(spec, ma_spec);
    //     for(int i = 1; i < multi_problem.numAgents(); i++){
    //         multi_problem.agent_properties[i].q_goal = multi_problem.agent_properties[0].q_goal;
    //     }


    //     SearchAndPlan search_plan_algo(multi_problem, multi_problem.numAgents());

    //     auto t0 = std::chrono::high_resolution_clock::now();
    //     amp::MultiAgentPath2D rovers_path = search_plan_algo.run();
    //     auto t1 = std::chrono::high_resolution_clock::now();
    //     double run_time = std::chrono::duration<double>(t1-t0).count();
    //     run_times.push_back(run_time);


    //     // amp::MultiAgentPath2D frontier_path;
    //     for(int i = 0; i < multi_problem.numAgents(); i++){
    //         // frontier_path.agent_paths.insert(frontier_path.agent_paths.begin(), rovers_path.agent_paths.back());
    //         rovers_path.agent_paths.pop_back();
    //     }
    //     if (rovers_path.agent_paths[0].waypoints.back() != multi_problem.agent_properties[0].q_goal){
    //         amp::Visualizer::makeFigure(search_plan_algo.getMapptr());
    //         amp::Visualizer::makeFigure(multi_problem, rovers_path);
    //         amp::Visualizer::saveFigures(true, "ARES_failure_case");
    //         continue;
    //     }
    //     bool success = amp::HW8::check(rovers_path, multi_problem);
    //     if(success){
    //         successful_runs += 1;
    //     }
    // }
    // all_successful_runs.push_back(successful_runs);
    // all_run_data.push_back(run_times);
    // amp::Visualizer::makeBoxPlot(all_run_data, {"-"}, "Run Time over 100 runs", "-" , "Time (s)");
    // amp::Visualizer::makeBarGraph(all_successful_runs, {"-"}, "Number of Successful Runs over 100 runs", "-", "Number of Successful Runs");



    // amp::HW2::check(rovers_path.agent_paths[0], problem);
    // amp::HW8::check(rovers_path, multi_problem);
    // amp::Visualizer::makeFigure(multi_problem, rovers_path);
    // amp::Visualizer::makeFigure(multi_problem, frontier_path);
    // amp::Visualizer::makeFigure(search_plan_algo.getMapptr());
    // amp::Visualizer::saveFigures(true, "ARES");


    // test lidar emulater
    // int cell_per_meter = 20;
    // MyLidarEmulateConstructor lidar_space((problem.x_max-problem.x_min)*cell_per_meter, (problem.y_max-problem.y_min)*cell_per_meter, problem);
    // auto t0 = std::chrono::high_resolution_clock::now();
    // lidar_space.construct4point1D(problem.q_init);
    // auto t1 = std::chrono::high_resolution_clock::now();
    // DEBUG("Total time: " << std::chrono::duration<double>(t1-t0).count() << " s");
    // amp::Visualizer::makeFigure(problem);
    // amp::Visualizer::makeFigure(lidar_space.getMapptr());
    // amp::Visualizer::saveFigures(true, "ARES");
    // return 0;
}