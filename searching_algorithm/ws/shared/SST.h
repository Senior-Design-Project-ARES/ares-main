#pragma once

#include "AMPCore.h"
#include "HelpfulClass.h"
#include <time.h>
#include <cmath>
#include "MyKinoRRT.h"
#include "tools/Usefull.h"

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
        SST(double delta_bn_, double delta_s_) : delta_bn(delta_bn_), delta_s(delta_s_) {};
        amp::Path planND(Eigen::VectorXd init_, Eigen::VectorXd goal_, amp::GridCSpace2D_T<int8_t> envMap);
        std::map<amp::Node, StateAndControl> getNodes(){
            return nodes;
        };
        std::shared_ptr<amp::Graph<double>> getGraphPtr(){
            return graphPtr;
        };
        bool collisionCheck(const StateAndControl& point, amp::GridCSpace2D_T<int8_t> envMap);
    private:
        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
        std::map<amp::Node, StateAndControl> nodes;
        std::vector<Neighborhood> neighborhoods;
        amp::Node bestFirstSelection(const Eigen::VectorXd& point, const std::map<amp::Node, StateAndControl>& nodes);
        StateAndControl extendSST(const amp::Node node);
        double delta_bn;
        double delta_s;
        int iteration = 5000;
        DifferentialDrive agent;
        double width = 0.2;  // Example width of the agent
        double height = 0.2; // Example height of the agent
        
};