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

struct Neighborhood {
    Eigen::VectorXd center;
    std::vector<amp::Node> nodes_in_neighborhood;
};

class SST {
    public:
        SST(double delta_bn_, double delta_s_);
        amp::Path planND(Eigen::VectorXd init_, Eigen::VectorXd goal_);

    private:
        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
        std::map<amp::Node, StateAndControl> nodes;
        std::vector<Neighborhood> neighborhoods;
        amp::Node bestFirstSelection(const Eigen::Vector2d& point, const std::map<amp::Node, StateAndControl>& nodes);
        StateAndControl extendSST(const amp::Node node);
        double delta_bn;
        double delta_s;
        int iteration = 10000;
};