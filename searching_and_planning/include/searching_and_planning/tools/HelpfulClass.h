#pragma once
#include <cstdint>
#include "searching_and_planning/tools/MyPath.h"
#include "searching_and_planning/tools/ConfigurationSpace.h"
#include "searching_and_planning/core/UsefulMacros.h"

// #define GAP 0.05
// #define GAPDECEN 0.1

template <typename P>
class BaseCollisionChecker {
    public:
        virtual bool isCollide(const P& point_) {
            (void)point_;
            return false;
        }
        virtual bool isCollide2P(const P& point1_, const P& point2_) {
            (void)point1_;
            (void)point2_;
            return false;
        }

        std::vector<std::pair<double,double>> getBounds() {return bounds;}
        void setBounds(std::vector<std::pair<double,double>> bounds_) {bounds = bounds_;}

    private:
        std::vector<std::pair<double,double>> bounds;
};

// class Point2DCollisionChecker : public BaseCollisionChecker<Eigen::VectorXd>{
//     public:
//         Point2DCollisionChecker(const amp::Environment2D& enviroument_);

//         bool isCollide(const Eigen::VectorXd& point_) override;
//         bool isCollide2P(const Eigen::VectorXd& point1_, const Eigen::VectorXd& point2_) override;

//         bool thisObstacle(int ob_idx_, const Eigen::VectorXd& point_);

//     private:
//         const amp::Environment2D& env;
// };

class Point2DCollisionCheckerGrid : public BaseCollisionChecker<Eigen::VectorXd>{
    public:
        Point2DCollisionCheckerGrid(const amp::GridCSpace2D_T<int8_t>& map_);

        bool isCollide(const Eigen::VectorXd& point_) override;
        bool isCollide2P(const Eigen::VectorXd& point1_, const Eigen::VectorXd& point2_) override;

    private:
        const amp::GridCSpace2D_T<int8_t>& map;
};

class MultiAgentPoint2DCollisionCheckerGrid{
    public:
        MultiAgentPoint2DCollisionCheckerGrid(const std::vector<amp::GridCSpace2D_T<int8_t>>& maps_)
        : maps(maps_){}

        bool isCollide(int agent_idx_, const Eigen::VectorXd& point_);
        bool isCollide2P(int agent_idx_, const Eigen::VectorXd& point1_, const Eigen::VectorXd& point2_);

    private:
        const std::vector<amp::GridCSpace2D_T<int8_t>>& maps;
};

// class MultiAgentDisk2DCollisionChecker : public BaseCollisionChecker<Eigen::VectorXd>{
//     public:
//         MultiAgentDisk2DCollisionChecker(const amp::MultiAgentProblem2D& problem);

//         bool isCollide(const Eigen::VectorXd& point_) override;
//         bool isCollide2P(const Eigen::VectorXd& point1_, const Eigen::VectorXd& point2_) override;

//         bool thisObstacle(int ob_idx_, const Eigen::Vector2d& point_);

//     private:
//         void decompose(std::vector<Eigen::Vector2d>& robots_location, const Eigen::VectorXd& point_);
//         bool checkEachGap(Eigen::VectorXd point, int sec_num);

//         const amp::MultiAgentProblem2D& problem;
//         std::vector<double> robots_radius;
// };

// class MultiAgentDisk2DCollisionCheckerDecen : public BaseCollisionChecker<Eigen::VectorXd>{
//     public:
//         MultiAgentDisk2DCollisionCheckerDecen(const amp::MultiAgentProblem2D& problem, const amp::MultiAgentPath2D& exisiting_path);

//         bool isCollide(const Eigen::VectorXd& point_) override{return true;}
//         bool isCollideWithTime(const Eigen::VectorXd& point_, int time_step);
//         bool isCollide2P(const Eigen::VectorXd& point1_, const Eigen::VectorXd& point2_) override{return true;}
//         bool isCollide2PWithTime(const Eigen::VectorXd& point1_, const Eigen::VectorXd& point2_, int time_step);

//         bool thisObstacle(int ob_idx_, const Eigen::Vector2d& point_);

//     private:
//         void decompose(std::vector<Eigen::Vector2d>& robots_location, const Eigen::VectorXd& point_);

//         const amp::MultiAgentProblem2D& problem;
//         std::vector<double> robots_radius;
//         const amp::MultiAgentPath2D& exisiting_path;
//         int currect_agent;

// };

template <typename T>
class QuickSort{
    public:
        void run(std::vector<int>& arr, const std::unordered_map<int, T>& map, int low, int high){
            if (low < high) {
                // pi is the partition return index of pivot
                int pi = partition(arr, map, low, high);

                // recursion calls for smaller elements
                // and greater or equals elements
                run(arr, map, low, pi - 1);
                run(arr, map, pi + 1, high);
            }
        }

    private:
        int partition(std::vector<int>& arr, const std::unordered_map<int, T>& map, int low, int high){
            // choose the pivot
            double pivot = map.at(arr[high]);
        
            // undex of smaller element and indicates 
            // the right position of pivot found so far
            int i = low - 1;

            // Traverse arr[low..high] and move all smaller
            // elements on left side. Elements from low to 
            // i are smaller after every iteration
            for (int j = low; j <= high - 1; j++) {
                if (map.at(arr[j]) < pivot) {
                    i++;
                    std::swap(arr[i], arr[j]);
                }
            }
            
            // move pivot after smaller elements and
            // return its position
            std::swap(arr[i + 1], arr[high]);  
            return i + 1;
        }
};
