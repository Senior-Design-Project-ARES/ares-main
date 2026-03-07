#pragma once

#include <Eigen/Core>

struct Target{
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    bool found = false;
    bool have_plan = false;
};