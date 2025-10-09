#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"
#include "hw/HW6.h"
#include "tools/Usefull.h"
#include <cstdint>

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
        /// \brief Create C-Space for point agent
        /// \param cells_x_dim: number of cells for x dim
        /// \param cells_y_dim: number of cells for y dim
        /// \returns constructed object
        MyPointAgentCSConstructor(std::size_t cells_x_dim, std::size_t cells_y_dim) 
        : cells_x_dim(cells_x_dim), cells_y_dim(cells_y_dim) {}

        /// \brief Create the FrontExpl object
        /// \param env: the original enviroument for cspace
        /// \returns GridSpace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::Environment2D& env) override;

        std::vector<int8_t> construct1D(const amp::Environment2D& env);

        std::size_t cells_x(){
            return cells_x_dim;
        }
        std::size_t cells_y(){
            return cells_y_dim;
        }

    private:
        std::size_t cells_x_dim;
        std::size_t cells_y_dim;
};

class MyLidarEmulateConstructor : public MyPointAgentCSConstructor{
    public:
        /// \brief Create C-Space for point agent
        /// \param cells_x_dim: number of cells for x dim
        /// \param cells_y_dim: number of cells for y dim
        /// \returns constructed object
        MyLidarEmulateConstructor(std::size_t cells_x_dim, std::size_t cells_y_dim, const amp::Environment2D& env) 
        : MyPointAgentCSConstructor(cells_x_dim, cells_y_dim),
        env(env),
        map(cells_x_dim, cells_y_dim, env.x_min, env.x_max, env.y_min, env.y_max, -1){}
        
        const amp::GridCSpace2D_T<int8_t>& construct4point(Eigen::Vector2d location);
        std::vector<int8_t> construct4point1D(Eigen::Vector2d location);
        std::vector<int8_t> getMap1D();
        const amp::GridCSpace2D_T<int8_t>& getMapptr();
        void addFrontierToMap(const std::vector<Eigen::Vector2i>& frontier_pts);
        void removeFrontierFromMap();

    private:
        const amp::Environment2D& env;
        amp::GridCSpace2D_T<int8_t> map;
        std::vector<Eigen::Vector2i> last_frontier;
};