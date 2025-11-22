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

        inline bool all(const Eigen::Vector2d& point);
        inline bool thisObstacle(int ob_idx,const Eigen::Vector2d& point);
        inline Eigen::Vector2d firstCollisionPoint(const Eigen::Vector2d& start, const Eigen::Vector2d& direction);

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

inline Eigen::Vector2d check_collision_enviroument::firstCollisionPoint(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2Mv1){
    Eigen::Vector2d v2 = v1 + v2Mv1;
    std::vector<Eigen::Vector2d> collision_points;

    for(int obstacle_idx = 0; obstacle_idx < env.obstacles.size(); ++obstacle_idx){
        const amp::Obstacle2D& obstacle = env.obstacles[obstacle_idx];
        int n = obstacle.verticesCCW().size();
        for(int vertices_idx = 0; vertices_idx < n; ++vertices_idx){
            const Eigen::Vector2d& v3 = obstacle.verticesCCW()[vertices_idx];
            const Eigen::Vector2d& v4 = obstacle.verticesCCW()[(vertices_idx + 1) % n];

            double denom = (v1(0) - v2(0)) * (v3(1) - v4(1)) - (v1(1) - v2(1)) * (v3(0) - v4(0));
            if(std::abs(denom) < 1e-10) continue; // Parallel lines

            double x = ((v1(0)*v2(1) - v1(1)*v2(0)) * (v3(0) - v4(0)) - (v1(0) - v2(0)) * (v3(0)*v4(1) - v3(1)*v4(0))) / denom;
            double y = ((v1(0)*v2(1) - v1(1)*v2(0)) * (v3(1) - v4(1)) - (v1(1) - v2(1)) * (v3(0)*v4(1) - v3(1)*v4(0))) / denom;

            Eigen::Vector2d intersection_point = Eigen::Vector2d(x, y);
            Eigen::Vector2d edge_dir = intersection_point - v1;
            edge_dir.normalize();
            double dot_product = edge_dir.dot(v2Mv1);

            if(dot_product < 0) continue; // Intersection is in the opposite direction

            if(thisObstacle(obstacle_idx, intersection_point+ edge_dir * 1e-6)){
                collision_points.push_back(intersection_point);
            }
            
        }
    }

    if(collision_points.empty()){
        return Eigen::Vector2d::Zero(); // No collision
    }

    double min_distance = std::numeric_limits<double>::infinity();
    Eigen::Vector2d closest_point;
    for(const auto& point : collision_points){
        double distance = (point - v1).norm();
        if(distance < min_distance){
            min_distance = distance;
            closest_point = point;
        }
    }
    return closest_point;

}

inline bool check_collision_enviroument::all(const Eigen::Vector2d& point) {
    double cross_product;
    bool possible_collision = 1;
    for (int ob_idx = 0; ob_idx < env.obstacles.size(); ++ob_idx) {
        if (thisObstacle(ob_idx, point)){
            return true; // Point is inside this polygons
        }
    }
    return false; // Point is outside all polygons
}


inline bool check_collision_enviroument::thisObstacle(int ob_idx,const Eigen::Vector2d& point){
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


