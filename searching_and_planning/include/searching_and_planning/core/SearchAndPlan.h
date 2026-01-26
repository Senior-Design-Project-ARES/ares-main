#pragma once

#include "frontierSearching.h"
#include "Path.h"
#include "MySamplingBasedPlanners.h"
#include "Logging.h"
#include "HelpfulStructs.h"

namespace ares {
class SearchAndPlanCore
{
    public:
        SearchAndPlanCore(const int& map_width, const int& map_height, const std::pair<double, double>& x, const std::pair<double, double>& y, const std::vector<int8_t>& FE_map, const Target& target);
        void updateGrid();
        void addPathObstacles2Grid(const std::vector<Path2D>& paths);
        ares::Path2D runSingle(const Eigen::Vector2d current_location, const std::vector<Path2D>& other_rover_paths);
        ares::Path2D runWithGoal(const Eigen::Vector2d current_location, const Eigen::Vector2d goal_location, const std::vector<Path2D>& other_rover_paths);

    private:
        const int map_width;
        const int map_height;
        const double resolution;
        const Eigen::Vector2d& origin;
        const std::vector<int8_t>& FE_map;
        FrontExpl front_expl;
        amp::GridCSpace2D_T<int8_t> grid_map;
        const Target& target;

        Eigen::Vector2d nextPoint(std::vector<std::pair<Eigen::Vector2d, int>>& point_of_interest, Eigen::Vector2d location);
};
} // namespace ares