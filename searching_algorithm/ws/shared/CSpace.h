#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"
#include "hw/HW6.h"
#include "tools/Usefull.h"

// Derive the amp::GridCSpace2D class and override the missing method
class MyGridCSpace2D : public amp::GridCSpace2D {
    public:
        MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) // Call base class constructor
        {}

        // Override this method for determining which cell a continuous point belongs to
        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;

};

// Derive the HW4 ManipulatorCSConstructor class and override the missing method
class MyManipulatorCSConstructor : public amp::ManipulatorCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyManipulatorCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}

        // Override this method for computing all of the boolean collision values for each cell in the cspace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;

        bool check_collision(const amp::Environment2D& env, Eigen::Vector2d point);
        bool check_collision_between_point(const amp::Environment2D& env, Eigen::Vector2d point1, Eigen::Vector2d point2, double critical);

    private:
        std::size_t m_cells_per_dim;
};

class MyPointAgentCSConstructor : public amp::PointAgentCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyPointAgentCSConstructor(std::size_t cells_x_dim, std::size_t cells_y_dim) 
        : cells_x_dim(cells_x_dim), cells_y_dim(cells_y_dim) {}

        // Override this method for computing all of the boolean collision values for each cell in the cspace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::Environment2D& env) override;

    private:
        std::size_t cells_x_dim;
        std::size_t cells_y_dim;
};