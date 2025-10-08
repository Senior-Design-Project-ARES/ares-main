#pragma once

#include "tools/Obstacle.h"
#include "tools/Environment.h"
#include <chrono>

class check_collision_problem{
    public:
        check_collision_problem(const amp::Problem2D& problem) :
            problem(problem){}

        inline bool all(Eigen::Vector2d point);
        inline bool thisObstacle(int ob_idx, Eigen::Vector2d point);

    private:
        const amp::Problem2D& problem;
};

class check_collision_enviroument{
    public:
        check_collision_enviroument(const amp::Environment2D& env) :
            env(env){}

        inline bool all(Eigen::Vector2d point);
        inline bool thisObstacle(int ob_idx, Eigen::Vector2d point);

    private:
        const amp::Environment2D& env;
};

inline bool check_collision_problem::all(Eigen::Vector2d point) {
    double cross_product;
    bool possible_collision = 1;
    for (int ob_idx = 0; ob_idx < problem.obstacles.size(); ++ob_idx) {
        if (thisObstacle(ob_idx, point)){
            return true; // Point is inside this polygons
        }
    }
    return false; // Point is outside all polygons
}

inline bool check_collision_problem::thisObstacle(int ob_idx, Eigen::Vector2d point){
    amp::Obstacle2D obstacle = problem.obstacles[ob_idx];
    double cross_product;

    int n = obstacle.verticesCCW().size();
    for (int vertix_idx = 0; vertix_idx < n; ++vertix_idx) {
        const Eigen::Vector2d& v1 = obstacle.verticesCCW()[vertix_idx];
        const Eigen::Vector2d& v2 = obstacle.verticesCCW()[(vertix_idx + 1) % n];
        Eigen::Vector2d edge = v1 - v2;
        Eigen::Vector2d point_to_v1 = point - v1;
        cross_product = edge(0) * point_to_v1(1) - edge(1) * point_to_v1(0);
        if (cross_product > 0){
            return false; // Point is outside of this obstacle
        }
    }
    return true; // Point is inside this obstacle
}

inline bool check_collision_enviroument::all(Eigen::Vector2d point) {
    double cross_product;
    bool possible_collision = 1;
    for (int ob_idx = 0; ob_idx < env.obstacles.size(); ++ob_idx) {
        if (thisObstacle(ob_idx, point)){
            return true; // Point is inside this polygons
        }
    }
    return false; // Point is outside all polygons
}


inline bool check_collision_enviroument::thisObstacle(int ob_idx, Eigen::Vector2d point){
    amp::Obstacle2D obstacle = env.obstacles[ob_idx];
    double cross_product;

    int n = obstacle.verticesCCW().size();
    for (int vertix_idx = 0; vertix_idx < n; ++vertix_idx) {
        const Eigen::Vector2d& v1 = obstacle.verticesCCW()[vertix_idx];
        const Eigen::Vector2d& v2 = obstacle.verticesCCW()[(vertix_idx + 1) % n];
        Eigen::Vector2d edge = v1 - v2;
        Eigen::Vector2d point_to_v1 = point - v1;
        cross_product = edge(0) * point_to_v1(1) - edge(1) * point_to_v1(0);
        if (cross_product > 0){
            return false; // Point is outside of this obstacle
        }
    }
    return true; // Point is inside this obstacle
}


