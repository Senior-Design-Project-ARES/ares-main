#include "frontierSearching.h"
#include "CSpace.h"
#include "AMPCore.h"
#include "MySamplingBasedPlanners.h"
#include "SST.h"
#include "MyKinoRRT.h"

#define CELL_PER_METER 20        // in cell per meter

class SearchAndPlan{
    public:
        SearchAndPlan(const amp::MultiAgentProblem2D& problem_, const int num_rover_);

        amp::MultiAgentPath2D runSingle(int rover_id);
        amp::MultiAgentPath2D run();
        std::pair<amp::MultiAgentPath2D, std::vector<double>> runWithTime();

        Eigen::Vector2d nextPoint(std::vector<std::pair<Eigen::Vector2d, int>>& point_of_interest, Eigen::Vector2d location);
        Eigen::Vector2d nextPoint_closest(std::vector<std::pair<Eigen::Vector2d, int>>& point_of_interest, Eigen::Vector2d location);
        Eigen::Vector2d nextPoint_largestFrontier(std::vector<std::pair<Eigen::Vector2d, int>>& point_of_interest, Eigen::Vector2d location);

        const amp::GridCSpace2D_T<int8_t>& getMapptr(){
            return C_space.getMapptr();
        };

        const amp::GridCSpace2D_T<int8_t>& getDiskMapptr(){
            return C_space.getDiskMapptr();
        };
        
        int state(Eigen::Vector2d);

    private:
        const amp::MultiAgentProblem2D& problem;
        const int num_rover;
        const int num_cells_x;
        const int num_cells_y;
        bool target_found = false;
        bool no_frountier_left = false;
        // MyLidarEmulateConstructor lidar_space;
        MyDiskAgentCS C_space;
        // const std::vector<int8_t>& map_1d;
        amp::MultiAgentPath2D rovers_paths;
        amp::MultiAgentPath2D frontier_paths;
        amp::MultiAgentPath2D active_paths;
        FrontExpl front_expl;
        // Point2DCollisionCheckerGrid collision_checker;
        std::vector<amp::GridCSpace2D_T<int8_t>*> multi_maps;
        std::vector<Point2DCollisionCheckerGrid*> multi_collision_checker;

        /// \brief Convert Eigen::VectorXd to Eigen::Vector2d
        /// \param vec: the Eigen::VectorXd to convert
        /// \returns Eigen::Vector2d
        Eigen::VectorXd eigen2dToEigenXd(const Eigen::Vector2d& vec){
            Eigen::VectorXd vec_xd(2);
            vec_xd(0) = vec(0);
            vec_xd(1) = vec(1);
            return vec_xd;
        }

        void updateMultiMap(int rover_id);
};