#pragma once

// This includes all of the necessary header files in the toolbox
// #include "HelpfulClass.h"
#include <time.h>
#include <cmath>
#include "searching_and_planning/tools/Graph.h"
#include "searching_and_planning/tools/MyPath.h"
#include "searching_and_planning/tools/HelpfulClass.h"
#include <Eigen/Core>
#include <map>

class MyGenericRRT {
    public:
        MyGenericRRT(double bias_, int iteration_, double step_size_) : bias(bias_), iteration(iteration_), step_size(step_size_) {}

        std::shared_ptr<ares::Graph<double>> getGraphPtr() { return graphPtr; }

        std::map<ares::Node , Eigen::VectorXd> getNodes() { return nodes; }

        ares::Path planND(Eigen::VectorXd init_, Eigen::VectorXd goal_, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_);

        // ares::Path planNDDecen(Eigen::VectorXd init_, Eigen::VectorXd goal_, MultiAgentDisk2DCollisionCheckerDecen& collision_checker_);

    private:
        std::shared_ptr<ares::Graph<double>> graphPtr = std::make_shared<ares::Graph<double>>();
        std::map<ares::Node , Eigen::VectorXd> nodes;
        Eigen::VectorXd generatePoint(const std::vector<std::pair<double, double>>& bounds);
        ares::Node  closestPoint(const Eigen::VectorXd& point);
        Eigen::VectorXd extendRRT(const Eigen::VectorXd& point, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_);
        // Eigen::VectorXd extendRRTDecen(const Eigen::VectorXd& point, MultiAgentDisk2DCollisionCheckerDecen& collision_checker_);
        bool checkDistance(Eigen::VectorXd direction, double requirement);
        double magnitude(Eigen::VectorXd vec);
        double bias;
        int iteration;
        double step_size;
};

// class MyRRT : public GoalBiasRRT2D, public MyGenericRRT {
//     public:
//         MyRRT(double bias_, int iteration_, double step_size_) : MyGenericRRT(bias_, iteration_, step_size_) {}
//         virtual amp::Path2D plan(const amp::Problem2D& problem_) override; 

//         std::map<amp::Node, Eigen::Vector2d> getNodes2D();
// };
