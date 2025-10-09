/// \file
/// \brief This node causes the robot to move autonomously via Frontier Exploration

#include <vector>
#include <iostream>
#include <cmath> 
#include <typeinfo>
#include <cstdint>
#include <Eigen/Dense>


class FrontExpl
{
    public:
        /// \brief Create the FrontExpl object
        /// \param map_width: Number of cells in the x direction
        /// \param map_height: Number of cells in the y direction
        /// \param resolution: The map resolution in meters/cell
        /// \param origin: The world coordinates of the map's origin
        /// \param FE0_map: The occupancy grid data from the map (do not need to update every time the map is updated, it is pass by reference)
        /// \returns constructed object
        FrontExpl(int map_width, int map_height, double resolution, const Eigen::Vector2d& origin, const std::vector<int8_t>& FE0_map);

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
        std::vector<Eigen::Vector2d> get_frontiers(const Eigen::Vector2d& location);

        std::vector<Eigen::Vector2i> get_grid_frontiers() { return centroid_grid_pts; }

    private:
        const std::vector<int8_t>& FE0_map;
        Eigen::Vector2d point;
        Eigen::Vector2d robot0_pose_;
        Eigen::Vector2d origin;
        std::string map0_frame = "tb3_0/map";
        std::string body0_frame = "tb3_0/base_footprint";
        std::vector<signed int> edge0_vec, neighbor0_index, neighbor0_value;
        std::vector<unsigned int> centroids0, temp_group0;
        std::vector<double> centroid0_Xpts, centroid0_Ypts, dist0_arr, prev_cent_0x, prev_cent_0y;
        std::vector<Eigen::Vector2d> centroid_pts;
        std::vector<Eigen::Vector2i> centroid_grid_pts;
        int group0_c=0, prev_group0_c=0, centroid0=0, centroid0_index=0, move_to_pt=0, map_width=0, map_height=0, mark_edge=0, edge_index=0;
        double smallest = 9999999.0, dist0= 0.0, resolution = 0.0;
        bool unique_flag = true;
};