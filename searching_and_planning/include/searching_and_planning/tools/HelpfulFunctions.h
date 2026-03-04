#pragma once

#ifndef HELPFULFUNCTIONS_H
#define HELPFULFUNCTIONS_H

#include <Eigen/Core>

inline Eigen::VectorXd eigen2dToEigenXd(const Eigen::Vector2d& vec2d) {
    Eigen::VectorXd vecXd(2);
    vecXd(0) = vec2d(0);
    vecXd(1) = vec2d(1);
    return vecXd;
}

inline Eigen::Vector2d eigenXdToEigen2d(const Eigen::VectorXd& vecXd) {
    Eigen::Vector2d vec2d;
    vec2d(0) = vecXd(0);
    vec2d(1) = vecXd(1);
    return vec2d;
}

#endif // HELPFULFUNCTIONS_H