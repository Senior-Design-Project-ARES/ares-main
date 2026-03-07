/// \file
/// \brief This node causes the robot to move autonomously via Frontier Exploration

#include <vector>
#include <iostream>
#include <cmath> 
#include <typeinfo>
#include <cstdint>
#include <map>

#include <Eigen/Dense>
#include "searching_and_planning/tools/Logging.h"
#include "searching_and_planning/core/UsefulMacros.h"
#include "searching_and_planning/core/Config.h"

// Forward declaration so FrontExpl can hold containers of FrontNode
class FrontNode;


class FrontExpl
{
    public:
        /// \brief Create the FrontExpl object
        /// \param map_width: Number of cells in the x direction
        /// \param map_height: Number of cells in the y direction
        /// \param resolution: The map resolution in meters/cell
        /// \param origin: The world coordinates of the map's origin
        /// \param FE0_map: The occupancy grid data from the map (do not need to update every time the map is updated, it is pass by reference)
        /// \param config: Configuration object passed from parent planner
        /// \returns constructed object
        FrontExpl(int map_width, int map_height, double resolution, const Eigen::Vector2d& origin, const std::vector<int8_t>& FE0_map, const searching_and_planning::Config& config);

        /// \brief Stores a vector of index values of the 8 cell neighborhood relative to the input cell
        /// \param cell - map cell
        /// \returns nothing
        void neighborhood(int cell);

        /// \brief Stores a vector of frontier edges
        /// \returns nothing
        void find_all_edges();

        /// \brief Compares two cells to see if the are identical or unique
        /// \param - curr_cell: The current cell being evaluated
        /// \param - next_cell: The next cell to be evaluated
        /// \returns true or false
        bool check_edges(int curr_cell, int next_cell);

        /// \brief Given the frontier edges, group the neighboring edges into regions
        /// \returns nothing
        void find_regions();

        /// \brief Given the regions, find the centroids of each region
        /// \returns nothing
        void find_centroids();

        // /// \brief Finds the transform between the map frame and the robot's base_footprint frame
        // /// \returns nothing
        // void find_transform();

        /// \brief Given a centroid cell, convert the centroid to x-y coordinates in the map frame and determine its distance from the robot
        /// \returns nothing
        void centroid_index_to_point();

        /// \brief Given all the centroid distance values, find the closest centroid to move to
        /// \returns nothing
        void find_closest_centroid();

        /// \brief Given the frontier edges, convert the cell to x-y coordinates in the map frame and determine its distance from the robot
        /// \returns nothing
        void edge_index_to_point();

        /// \brief Calls all other functions to find frontier edges, regions and a goal to move to. Then uses the action server to move to that goal
        /// \returns nothing
        std::vector<std::pair<Eigen::Vector2d, int>> run();

        std::vector<Eigen::Vector2i> getCentroidsGrid() { return centroid_grid_pts; }

    private:
        const searching_and_planning::Config& config;
        Eigen::Vector2d point;
        // Eigen::Vector2d robot0_pose_;
        std::string map0_frame = "tb3_0/map";
        std::string body0_frame = "tb3_0/base_footprint";
        std::vector<signed int> edge0_vec, neighbor0_index, neighbor0_value;
        std::vector<unsigned int> centroids0, temp_group0;
        std::vector<double> centroid0_Xpts, centroid0_Ypts, dist0_arr;
        // std::vector<double> prev_cent_0x, prev_cent_0y;
        std::vector<std::pair<Eigen::Vector2d, int>> centroid_pts;
        std::vector<Eigen::Vector2i> centroid_grid_pts;
        std::vector<int> points_in_regions;
        std::vector<std::pair<std::vector<FrontNode>, bool>> frontier_regions;
        // int group0_c=0, prev_group0_c=0;
        int centroid0=0, centroid0_index=0, move_to_pt=0, map_width=0, map_height=0, mark_edge=0, edge_index=0;
        const std::vector<int8_t>& FE0_map;
        double smallest = 9999999.0, dist0= 0.0, resolution = 0.0;
        Eigen::Vector2d origin;
        bool unique_flag = true;
};

class FrontNode{
    public:
        FrontNode(int index): index(index) {}

        int getIndex() const { return index; }

        bool setFrontierNeighbors(int index){
            if (frontier_neighbors.first == 0)
                frontier_neighbors.first = index;
            else if (frontier_neighbors.second == 0)
                frontier_neighbors.second = index;
            else
                return false;
            return true;
        }
        void setFrontierNeighbors(int place, int index){
            if (place == 1)
                frontier_neighbors.first = index;
            else if (place == 2)
                frontier_neighbors.second = index;
        }
        // Accessor for frontier neighbors so external code can read them
        std::pair<int,int> getFrontierNeighbors() const { return frontier_neighbors; }
    
    private:
        int index;
        std::pair<int, int> frontier_neighbors = {0, 0};
};