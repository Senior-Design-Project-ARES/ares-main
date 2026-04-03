#pragma once

#include "searching_and_planning/core/frontierSearching.h"
#include "searching_and_planning/tools/MyPath.h"
#include "searching_and_planning/core/MySamplingBasedPlanners.h"
#include "searching_and_planning/tools/Logging.h"
#include "searching_and_planning/tools/HelpfulStructs.h"
#include "searching_and_planning/core/UsefulMacros.h"
#include "searching_and_planning/core/Config.h"
#include "searching_and_planning/tools/HelpfulFunctions.h"

namespace ares {
class SearchAndPlanCore
{
    public:
        SearchAndPlanCore(const size_t& map_width, const size_t& map_height, const std::pair<double, double>& x, const std::pair<double, double>& y, const std::vector<int8_t>& FE_map, const Target& target, const searching_and_planning::Config& config);
        void updateGrid();
        void addPathObstacles2Grid(const std::vector<Path2D>& paths);
        ares::Path2D runSingle(const Eigen::Vector2d current_location, const std::vector<Path2D>& other_rover_paths);
        ares::Path2D runWithGoal(const Eigen::Vector2d current_location, const Eigen::Vector2d goal_location, const std::vector<Path2D>& other_rover_paths);

    private:
        const searching_and_planning::Config& config;
        const size_t map_width;
        const size_t map_height;
        const double resolution;
        const Eigen::Vector2d origin;
        const std::vector<int8_t>& FE_map;
        FrontExpl front_expl;
        amp::GridCSpace2D_T<int8_t> grid_map;
        const Target& target;

        Eigen::Vector2d nextPoint(std::vector<std::pair<Eigen::Vector2d, int>>& point_of_interest, Eigen::Vector2d location);
};
} // namespace ares