#pragma once

#include <Eigen/Core>

struct Target{
    Eigen::Vector2d position;
    bool found = false;
    bool have_plan = false;
};