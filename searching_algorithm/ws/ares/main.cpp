#include "AMPCore.h"

#include "searching.h"
#include "CSpace.h"

#define CELL_PER_METER 5        // in cell per meter

int main(int argc, char** argv){
    amp::Environment2D env;
    std::vector<Eigen::Vector2d> vertices_cww;
    vertices_cww.push_back(Eigen::Vector2d(3.0, 2.0));
    vertices_cww.push_back(Eigen::Vector2d(6.0, 2.0));
    vertices_cww.push_back(Eigen::Vector2d(6.0, 8.0));
    vertices_cww.push_back(Eigen::Vector2d(3.0, 8.0));

    amp::Obstacle2D obstacle_1(vertices_cww);
    env.obstacles.push_back(obstacle_1);

    const int num_cells_x = (env.x_max-env.x_min)*CELL_PER_METER;
    const int num_cells_y = (env.y_max-env.y_min)*CELL_PER_METER;

    Eigen::Vector2d robot_pose_1 (1.5, 5.0);
    Eigen::Vector2d robot_pose_2 (2.0, 8.0);

    MyLidarEmulateConstructor lidar_space(num_cells_x, num_cells_y, env);
    lidar_space.construct4point1D(robot_pose_1);
    std::vector<int8_t> map = lidar_space.construct4point1D(robot_pose_2);
    FrontExpl front_expl(num_cells_x, num_cells_y, 1.0/CELL_PER_METER, Eigen::Vector2d(env.x_min, env.y_min), map);
    std::vector<Eigen::Vector2d> points = front_expl.run();

    std::vector<Eigen::Vector2i> grid_points = front_expl.getCentroidsGrid();
    lidar_space.addFrontierToMap(grid_points);
    

    amp::Visualizer::makeFigure(lidar_space.getMapptr());

    amp::Visualizer::saveFigures(true, "ARES");

    return 0;
}