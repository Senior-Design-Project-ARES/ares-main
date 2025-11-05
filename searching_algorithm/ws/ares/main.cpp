#include "AMPCore.h"

#include "SearchAndPlan.h"
#include "tools/Environment.h"
#include "hw/HW2.h"
#include "hw/HW8.h"

// #define CELL_PER_METER         // in cell per meter

int main(int argc, char** argv){
    amp::RNG::seed(amp::RNG::randiUnbounded());
    // amp::Environment2D env;
    // std::vector<Eigen::Vector2d> vertices_cww;
    // vertices_cww.push_back(Eigen::Vector2d(1.0, 1.0));
    // vertices_cww.push_back(Eigen::Vector2d(1.3, 1.0));
    // vertices_cww.push_back(Eigen::Vector2d(1.3, 1.3));
    // vertices_cww.push_back(Eigen::Vector2d(1.0, 1.3));

    // amp::Obstacle2D obstacle_1(vertices_cww);
    // env.obstacles.push_back(obstacle_1);
    // const int num_cells_x = (env.x_max-env.x_min)*CELL_PER_METER;
    // const int num_cells_y = (env.y_max-env.y_min)*CELL_PER_METER;

    amp::Random2DEnvironmentSpecification spec;
    amp::RandomCircularAgentsSpecification ma_spec;
    spec.x_max = 22.0;
    spec.y_max = 9.0;
    ma_spec.n_agents = 3;
    ma_spec.max_agent_radius = 0.15;
    ma_spec.min_agent_radius = 0.15;
    spec.n_obstacles = 60*2;
    spec.max_obstacle_region_radius = 0.8;
    spec.path_clearance = 0.3*1.2;
    // amp::Problem2D problem = amp::EnvironmentTools::generateRandomPointAgentProblem(spec);
    amp::MultiAgentProblem2D multi_problem;
    multi_problem = amp::EnvironmentTools::generateRandomMultiAgentProblem(spec, ma_spec);
    for(int i = 1; i < multi_problem.numAgents(); i++){
        multi_problem.agent_properties[i].q_goal = multi_problem.agent_properties[0].q_goal;
    }
    // DEBUG("Starting ARES planning....");
    SearchAndPlan search_plan_algo(multi_problem, multi_problem.numAgents());
    amp::MultiAgentPath2D rovers_path = search_plan_algo.run();
    amp::MultiAgentPath2D frontier_path;
    for(int i = 0; i < multi_problem.numAgents(); i++){
        frontier_path.agent_paths.insert(frontier_path.agent_paths.begin(), rovers_path.agent_paths.back());
        rovers_path.agent_paths.pop_back();
    }

    

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
    amp::HW8::check(rovers_path, multi_problem);
    amp::Visualizer::makeFigure(multi_problem, rovers_path);
    amp::Visualizer::makeFigure(multi_problem, frontier_path);
    amp::Visualizer::makeFigure(search_plan_algo.getMapptr());
    amp::Visualizer::saveFigures(true, "ARES");


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
    return 0;
}