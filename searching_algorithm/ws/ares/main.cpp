#include "AMPCore.h"

#include "SearchAndPlan.h"
#include "tools/Environment.h"

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
    spec.n_obstacles = 60;
    spec.max_obstacle_region_radius = 0.8;
    spec.path_clearance = 0.3;
    amp::Problem2D problem = amp::EnvironmentTools::generateRandomPointAgentProblem(spec);

    SearchAndPlan search_plan_algo(problem, 1);

    amp::MultiAgentPath2D rovers_path = search_plan_algo.run();    

    amp::Visualizer::makeFigure(problem, rovers_path.agent_paths[0]);
    amp::Visualizer::makeFigure(problem, rovers_path.agent_paths[1]);
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