#pragma once

#include "AMPCore.h"
#include "HelpfulClass.h"
#include <time.h>
#include <cmath>

struct StateAndControl {
    Eigen::VectorXd state;
    Eigen::VectorXd control;
    double cost;
    bool active = true;
};

struct neighborhood {
    Eigen::VectorXd center;
    std::vector<amp::Node> nodes_in_neighborhood;
};

class SST {
    public:
        SST(double delta_bn_, double delta_s_);
        amp::Path planND(Eigen::VectorXd init_, Eigen::VectorXd goal_, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_);

    private:
        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
        std::map<amp::Node, StateAndControl> nodes;
        std::vector<neighborhood> neighborhoods;
        Eigen::VectorXd generatePoint(const std::vector<std::pair<double, double>>& bounds);
        amp::Node closestPoint(const Eigen::VectorXd& point);
        Eigen::VectorXd extendSST(const Eigen::VectorXd& point, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_);
        bool checkDistance(Eigen::VectorXd direction, double requirement);
        double magnitude(Eigen::VectorXd vec);
        double delta_bn;
        double delta_s;
        int iteration = 10000;
};