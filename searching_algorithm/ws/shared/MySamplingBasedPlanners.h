#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "HelpfulClass.h"
#include <time.h>
#include <cmath>
// #include "MyAStar.h"

// Include the correct homework headers
#include "hw/HW7.h"

// class MyGenericPRM {
//     public:
//         MyGenericPRM(int num_nodes_, int k_neighbors_, double radius_, bool smooth_);
//         std::shared_ptr<amp::Graph<double>> getGraphPtr() { return graphPtr; }
//         std::map<amp::Node, Eigen::VectorXd> getNodes() { return nodes; }
//         void createGraph(BaseCollisionChecker<Eigen::VectorXd>& collision_checker_);
//         amp::Path planND(Eigen::VectorXd init_, Eigen::VectorXd goal_, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_);

//     private:
//         std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
//         std::map<amp::Node, Eigen::VectorXd> nodes;
//         std::unordered_map<amp::Node, double> findClosest(Eigen::VectorXd location, int num_of_closest_);
//         void connectClosest(amp::Node node_idx, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_);
//         amp::Path smoothPath(amp::Path, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_);
//         int num_nodes;
//         int k_neighbors;
//         double radius;
//         bool smooth = false;

// };

// class MyPRM : public amp::PRM2D, public MyGenericPRM {
//     public:
//         MyPRM(int num_nodes_, int k_neighbors_, double radius_, bool smooth_);
//         virtual amp::Path2D plan(const amp::Problem2D& problem_) override;

//         std::map<amp::Node, Eigen::Vector2d> getNodes2D();
// };

class MyGenericRRT {
    public:
        MyGenericRRT(double bias_, int iteration_, double step_size_) : bias(bias_), iteration(iteration_), step_size(step_size_) {}
        std::shared_ptr<amp::Graph<double>> getGraphPtr() { return graphPtr; }
        std::map<amp::Node, Eigen::VectorXd> getNodes() { return nodes; }
        amp::Path planND(Eigen::VectorXd init_, Eigen::VectorXd goal_, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_);
        amp::Path planNDDecen(Eigen::VectorXd init_, Eigen::VectorXd goal_, MultiAgentDisk2DCollisionCheckerDecen& collision_checker_);

    private:
        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
        std::map<amp::Node, Eigen::VectorXd> nodes;
        Eigen::VectorXd generatePoint(const std::vector<std::pair<double, double>>& bounds);
        amp::Node closestPoint(const Eigen::VectorXd& point);
        Eigen::VectorXd extendRRT(const Eigen::VectorXd& point, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_);
        Eigen::VectorXd extendRRTDecen(const Eigen::VectorXd& point, MultiAgentDisk2DCollisionCheckerDecen& collision_checker_);
        bool checkDistance(Eigen::VectorXd direction, double requirement);
        double magnitude(Eigen::VectorXd vec);
        double bias;
        int iteration;
        double step_size;
};

class MyRRT : public amp::GoalBiasRRT2D, public MyGenericRRT {
    public:
        MyRRT(double bias_, int iteration_, double step_size_) : MyGenericRRT(bias_, iteration_, step_size_) {}
        virtual amp::Path2D plan(const amp::Problem2D& problem_) override; 

        std::map<amp::Node, Eigen::Vector2d> getNodes2D();
};
