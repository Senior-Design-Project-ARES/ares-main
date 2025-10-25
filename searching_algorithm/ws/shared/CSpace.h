/// \file CSpace.h
/// \brief This file contains custom Lidar emulator C-space

#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"
#include "hw/HW6.h"
#include "tools/Usefull.h"
#include <cstdint>
#include <Eigen/Geometry>

#define LIDARRADIUS 3
#define ANGLERESOLUSION 0.008
#define SAMPLE_POINT 10

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


/// \brief Create Lidar Emulate C-Space for point agent
/// \param cells_x_dim: number of cells for x dim
/// \param cells_y_dim: number of cells for y dim
/// \param env: the original enviroument for cspace
/// \returns constructed object
class MyLidarEmulateConstructor : public MyPointAgentCSConstructor{
    public:
        /// \brief Create C-Space for point agent
        /// \param cells_x_dim: number of cells for x dim
        /// \param cells_y_dim: number of cells for y dim
        /// \param env: the original enviroument for cspace
        /// \returns constructed object
        MyLidarEmulateConstructor(std::size_t cells_x_dim, std::size_t cells_y_dim, const amp::Environment2D& env) 
        : MyPointAgentCSConstructor(cells_x_dim, cells_y_dim),
        env(env),
        map(cells_x_dim, cells_y_dim, env.x_min, env.x_max, env.y_min, env.y_max, -1){}
        
        /// \brief Update space around a radius from unknow to free or occupied and return 2D GridCSpace
        /// \param location: the center point to update around
        /// \returns 2D GridCSpace
        const amp::GridCSpace2D_T<int8_t>& construct4point(Eigen::Vector2d location);

        /// \brief Mimic Lidar scan to update space around a radius from unknow to free or occupied and return 2D GridCSpace
        /// \param location: the center point to update around
        /// \returns 2D GridCSpace
        const amp::GridCSpace2D_T<int8_t>& lidarMimicConstruct4point(Eigen::Vector2d location);

        /// \brief Mimic Lidar scan to update space around a radius from unknow to free or occupied and return 2D GridCSpace
        /// \param location: the center point to update around
        /// \returns 2D GridCSpace
        const amp::GridCSpace2D_T<int8_t>& lidarMimicConstruct4point2(Eigen::Vector2d location);

        /// \brief Update space around a radius from unknow to free or occupied and return 1D map
        /// \param location: the center point to update around
        /// \returns 1D map
        const std::vector<int8_t>& construct4point1D(Eigen::Vector2d location);

        /// \brief Get the current 1D map
        /// \returns 1D map
        const std::vector<int8_t>& getMap1D(){return map_1D;};

        /// \brief Get the current 2D GridCSpace
        /// \returns 2D GridCSpace
        const amp::GridCSpace2D_T<int8_t>& getMapptr();

        /// \brief Add frontier points to map
        /// \param frontier_pts: the frontier points to add
        /// \returns nothing
        void addFrontierToMap(const std::vector<Eigen::Vector2i>& frontier_pts);

        /// \brief Remove all frontier points from map
        /// \returns nothing
        void removeFrontierFromMap();

        /// \brief Reset the entire map to unknown
        /// \returns nothing
        void reset();

    private:
        const amp::Environment2D& env;
        amp::GridCSpace2D_T<int8_t> map;
        std::vector<int8_t> map_1D;
        std::vector<Eigen::Vector2i> last_frontier;

        /// \brief Update cells around a point within a radius to a value
        /// \param location_check: the center point to update around
        /// \param value: the value to set the cells to
        /// \param radius: the radius around the center point to update
        /// \returns nothing
        void updateAroundPoint(const Eigen::Vector2d& location_check, int8_t value, double radius);

        /// \brief Check if the cell occupied by a location is in collision
        /// \param location_check: the location to check
        /// \param check_collision: the collision checker
        /// \returns true if in collision, false otherwise
        bool checkCellCollision(Eigen::Vector2d location_check, check_collision_enviroument& check_collision);

        /// \brief Get the state of a location in the map
        /// \param location: the location to check
        /// \returns state value
        int getState(const Eigen::Vector2d& location);
};