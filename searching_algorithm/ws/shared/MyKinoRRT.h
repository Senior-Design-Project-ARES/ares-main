#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include <algorithm>
#include <Eigen/Geometry>
#include "HelpfulClass.h"
#include "HelpfulFunction.h"

// Include the correct homework headers
#include "hw/HW9.h"
#include "hw/HW2.h"

#define EPSILON 1e-6
#define STEP_SIZE 0.04

struct MyEdge {
    Eigen::VectorXd control;
    double length;
    double dt;
};

class MyKinoRRT : public amp::KinodynamicRRT {
    public:
        MyKinoRRT(int max_iterations_, int control_sample_size_);
        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;

    private:
        std::shared_ptr<amp::Graph<MyEdge>> graphPtr = std::make_shared<amp::Graph<MyEdge>>();
        std::map<amp::Node, Eigen::VectorXd> nodes;
        const int max_iterations;
        const int control_sample_size;
        Eigen::VectorXd extendRRT(const amp::KinodynamicProblem2D& problem, Point2DCollisionChecker& checker, amp::DynamicAgent& agent, Eigen::VectorXd& rand_x);
        double distance(const amp::AgentType& agent_type, const Eigen::VectorXd& state1, const Eigen::VectorXd& state2);
        amp::KinoPath get_pre_plan_path();

        Eigen::Vector2d arm;
        Eigen::Vector2d to_center;
        Eigen::Rotation2D<double> rot1;
        Eigen::Rotation2D<double> rot2;
        Eigen::Rotation2D<double> rot3;
        Eigen::Rotation2D<double> rot4;
        // amp::KinoPath parking_path;
};  

class MyDynamicAgent : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {};
        void rungeKutta45(Eigen::VectorXd& state, const Eigen::VectorXd& control, double t);
        virtual Eigen::VectorXd dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) = 0;
};

class DifferentialDrive : public MyDynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {
            rungeKutta45(state, control, dt);
        }
        Eigen::VectorXd dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) override;
};

class MySingleIntegrator : public MyDynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {
            rungeKutta45(state, control, dt);
        }
        Eigen::VectorXd dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) override;
};

class MyFirstOrderUnicycle : public MyDynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {
            rungeKutta45(state, control, dt);
        }
        Eigen::VectorXd dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) override;
};

class MySecondOrderUnicycle : public MyDynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {
            rungeKutta45(state, control, dt);
        }
        Eigen::VectorXd dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) override;
};

class MySimpleCar : public MyDynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override {
            Eigen::VectorXd state_copy = state.head(state.size()-1);
            agent_dim.length = state(state.size()-1);
            rungeKutta45(state_copy, control, dt);
            for(int i = 0; i < state_copy.size(); i++){
                state(i) = state_copy(i);
            }
        }
        Eigen::VectorXd dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) override;

};