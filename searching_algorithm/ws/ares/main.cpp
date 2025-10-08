#include "searching.h"
#include "tools/ConfigurationSpace.h"
#include "tools/Environment.h"

#define CELL_PER_METER 4        // in cell per meter

int main(){
    amp::Environment2D env;
    std::vector<Eigen::Vector2d> vertices_cww;
    vertices_cww.push_back(Eigen::Vector2d(4.0, 4.0));
    vertices_cww.push_back(Eigen::Vector2d(6.0, 4.0));
    vertices_cww.push_back(Eigen::Vector2d(6.0, 6.0));
    vertices_cww.push_back(Eigen::Vector2d(4.0, 6.0));

    amp::Obstacle2D obstacle_1(vertices_cww);
    env.obstacles.push_back(obstacle_1);

    const int num_cells_x = (env.x_max-env.x_min)*CELL_PER_METER;
    const int num_cells_y = (env.y_max-env.y_min)*CELL_PER_METER;

    amp::DenseArray2D<int> map(num_cells_x,num_cells_y);


    return 0;
}