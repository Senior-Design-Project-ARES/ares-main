#include "frontierSearching.h"
#include "CSpace.h"
#include "AMPCore.h"
#include "MySamplingBasedPlanners.h"

#define CELL_PER_METER 20        // in cell per meter

class SearchAndPlan{
    public:
        SearchAndPlan(const amp::Problem2D& problem_, const int num_rover_);
        amp::MultiAgentPath2D run();
        Eigen::Vector2d nextPoint(std::vector<std::pair<Eigen::Vector2d, int>> point_of_interest, Eigen::Vector2d location);
        const amp::GridCSpace2D_T<int8_t>& getMapptr(){
            return lidar_space.getMapptr();
        };
        int state(Eigen::Vector2d);

    private:
        const amp::Problem2D& problem;
        const int num_rover;
        const int num_cells_x;
        const int num_cells_y;
        MyLidarEmulateConstructor lidar_space;

        /// \brief Convert Eigen::VectorXd to Eigen::Vector2d
        /// \param vec: the Eigen::VectorXd to convert
        /// \returns Eigen::Vector2d
        Eigen::VectorXd eigen2dToEigenXd(const Eigen::Vector2d& vec){
            Eigen::VectorXd vec_xd(2);
            vec_xd(0) = vec(0);
            vec_xd(1) = vec(1);
            return vec_xd;
        }
};