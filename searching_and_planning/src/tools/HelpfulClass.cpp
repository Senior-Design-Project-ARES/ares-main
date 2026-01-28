#include "searching_and_planning/tools/HelpfulClass.h"

// Point2DCollisionChecker::Point2DCollisionChecker(const amp::Environment2D& enviroument_)
// : env(enviroument_){
//     std::vector<std::pair<double,double>> bounds;
//     bounds.push_back({enviroument_.x_min, enviroument_.x_max});
//     bounds.push_back({enviroument_.y_min, enviroument_.y_max});
//     setBounds(bounds);
// }

// bool Point2DCollisionChecker::isCollide(const Eigen::VectorXd& point) {
//     for (int ob_idx = 0; ob_idx < env.obstacles.size(); ++ob_idx) {
//         if (thisObstacle(ob_idx, point)){
//             return true; // Point is inside this polygons
//         }
//     }
//     return false; // Point is outside all polygons
//     // Implementation
// }

// bool Point2DCollisionChecker::isCollide2P(const Eigen::VectorXd& point1_, const Eigen::VectorXd& point2_){
//     Eigen::VectorXd one_2_two = point2_ - point1_;
//     double distance = one_2_two.norm();
//     int sec_num = 1;
//     while(distance/sec_num > GAP){
//         for(int i = 0; i < sec_num; i++){
//             Eigen::VectorXd check_location = point1_ + one_2_two*(1+2*i)/(sec_num*2);
//             if (isCollide(check_location))
//             {
//                 return true;
//             }
//         }
//         sec_num = sec_num * 2;
//     }
//     return false;
// }

// bool Point2DCollisionChecker::thisObstacle(int ob_idx_, const Eigen::VectorXd& point_){
//     const amp::Obstacle2D& obstacle = env.obstacles[ob_idx_];
//     double cross_product;

//     int n = obstacle.verticesCCW().size();
//     for (int vertix_idx = 0; vertix_idx < n; ++vertix_idx) {
//         const Eigen::VectorXd& v1 = obstacle.verticesCCW()[vertix_idx];
//         const Eigen::VectorXd& v2 = obstacle.verticesCCW()[(vertix_idx + 1) % n];
//         Eigen::VectorXd edge = v1 - v2;
//         Eigen::VectorXd point_to_v1 = point_ - v1;
//         cross_product = edge(0) * point_to_v1(1) - edge(1) * point_to_v1(0);
//         if (cross_product > 0){
//             return false; // Point is outside of this obstacle
//         }
//     }
//     return true; // Point is inside this obstacle
// }

Point2DCollisionCheckerGrid::Point2DCollisionCheckerGrid(const amp::GridCSpace2D_T<int8_t>& map_)
: map(map_){
    std::vector<std::pair<double,double>> bounds;
    bounds.push_back({map.x0Bounds().first, map.x0Bounds().second});
    bounds.push_back({map.x1Bounds().first, map.x1Bounds().second});
    setBounds(bounds);
}

bool Point2DCollisionCheckerGrid::isCollide(const Eigen::VectorXd& point_){
    return(map.inCollision(point_(0), point_(1)));
}

bool Point2DCollisionCheckerGrid::isCollide2P(const Eigen::VectorXd& point1_, const Eigen::VectorXd& point2_){
    Eigen::VectorXd one_2_two = point2_ - point1_;
    double distance = one_2_two.norm();
    int sec_num = 1;
    while(distance/sec_num > GAP){
        for(int i = 0; i < sec_num; i++){
            Eigen::VectorXd check_location = point1_ + one_2_two*(1+2*i)/(sec_num*2);
            if (isCollide(check_location))
            {
                return true;
            }
        }
        sec_num = sec_num * 2;
    }
    return false;
}

bool MultiAgentPoint2DCollisionCheckerGrid::isCollide(int agent_idx_, const Eigen::VectorXd& point_){
    return(maps[agent_idx_].inCollision(point_(0), point_(1)));
}

bool MultiAgentPoint2DCollisionCheckerGrid::isCollide2P(int agent_idx_, const Eigen::VectorXd& point1_, const Eigen::VectorXd& point2_){
    Eigen::VectorXd one_2_two = point2_ - point1_;
    double distance = one_2_two.norm();
    int sec_num = 1;
    while(distance/sec_num > GAP){
        for(int i = 0; i < sec_num; i++){
            Eigen::VectorXd check_location = point1_ + one_2_two*(1+2*i)/(sec_num*2);
            if (isCollide(agent_idx_, check_location))
            {
                return true;
            }
        }
        sec_num = sec_num * 2;
    }
    return false;
}

// MultiAgentDisk2DCollisionChecker::MultiAgentDisk2DCollisionChecker(const amp::MultiAgentProblem2D& problem_)
// : problem(problem_){
//     std::vector<std::pair<double,double>> bounds;
//     for(const amp::CircularAgentProperties& agent : problem.agent_properties) {
//         bounds.push_back({problem.x_min, problem.x_max});
//         bounds.push_back({problem.y_min, problem.y_max});
//         robots_radius.push_back(agent.radius);
//         setBounds(bounds);
//         // LOG(agent.radius);
//     }

// }

// bool MultiAgentDisk2DCollisionChecker::isCollide(const Eigen::VectorXd& point){
//     std::vector<Eigen::Vector2d> robots_location;
//     decompose(robots_location, point);

//     for (int robots_idx; robots_idx< robots_location.size(); robots_idx++){
//         int num_of_sample = M_PI*pow(robots_radius[robots_idx]*1.2,2)*300;

//         // check if robot collide with enviroument
//         for (double theta = 0; theta< 2*M_PI; theta += 0.01){
//             Eigen::Vector2d point2check = robots_location[robots_idx] + Eigen::Vector2d(robots_radius[robots_idx]*cos(theta)*1.2, robots_radius[robots_idx]*sin(theta)*1.2);
//             for (int ob_idx = 0; ob_idx < problem.obstacles.size(); ++ob_idx) {
//                 if (thisObstacle(ob_idx, point2check)){
//                     return true; // Point is inside this polygons
//                 }
//             }
//         }


//         for (int sample_idx = 0; sample_idx < num_of_sample; sample_idx++){
//             double theta = double(rand())/RAND_MAX * 2*M_PI;
//             double arm = double(rand())/RAND_MAX * robots_radius[robots_idx]*1.1;
//             Eigen::Vector2d point2check = robots_location[robots_idx] + Eigen::Vector2d(arm*cos(theta), arm*sin(theta));

//             for (int ob_idx = 0; ob_idx < problem.obstacles.size(); ++ob_idx) {
//                 if (thisObstacle(ob_idx, point2check)){
//                     return true; // Point is inside this polygons
//                 }
//             }
//         }
        
//         // check if robot collide with other robot
//         for (int rest_robots_idx = 0; rest_robots_idx < robots_location.size(); rest_robots_idx++){
//             if (rest_robots_idx == robots_idx){
//                 continue;
//             }
//             double distance = (robots_location[robots_idx] - robots_location[rest_robots_idx]).norm();
//             if(distance <= (robots_radius[robots_idx] + robots_radius[rest_robots_idx])*1.1){
//                 return true; //Robot Collide
//             }
//         }
//     }
//     return false;
// }

// bool MultiAgentDisk2DCollisionChecker::isCollide2P(const Eigen::VectorXd& point1_, const Eigen::VectorXd& point2_){
//     Eigen::VectorXd one_2_two = point2_ - point1_;
//     // double distance = one_2_two.norm();
//     int sec_num = 1;
//     while(checkEachGap(one_2_two, sec_num)){
//         for(int i = 0; i < sec_num; i++){
//             Eigen::VectorXd check_location = point1_ + one_2_two*(1+2*i)/(sec_num*2);
//             if (isCollide(check_location))
//             {
//                 return true;
//             }
//         }
//         sec_num = sec_num * 2;
//     }
//     return false;
// }

// bool MultiAgentDisk2DCollisionChecker::checkEachGap(Eigen::VectorXd point, int sec_num){
//     for(int i = 0; i < problem.numAgents(); i++){
//         if((point(2*i)*point(2*i) + point(2*i+1)*point(2*i+1))/(sec_num*sec_num) >= GAP*GAP){
//             return true;
//         }
//     }
//     return false;
// }

// bool MultiAgentDisk2DCollisionChecker::thisObstacle(int ob_idx_, const Eigen::Vector2d& point_){
//     const amp::Obstacle2D& obstacle = problem.obstacles[ob_idx_];
//     double cross_product;

//     int n = obstacle.verticesCCW().size();
//     for (int vertix_idx = 0; vertix_idx < n; ++vertix_idx) {
//         const Eigen::Vector2d& v1 = obstacle.verticesCCW()[vertix_idx];
//         const Eigen::Vector2d& v2 = obstacle.verticesCCW()[(vertix_idx + 1) % n];
//         Eigen::Vector2d edge = v1 - v2;
//         Eigen::Vector2d point_to_v1 = point_ - v1;
//         cross_product = edge(0) * point_to_v1(1) - edge(1) * point_to_v1(0);
//         if (cross_product > 0){
//             return false; // Point is outside of this obstacle
//         }
//     }
//     return true; // Point is inside this obstacle
// }

// void MultiAgentDisk2DCollisionChecker::decompose(std::vector<Eigen::Vector2d>& robots_location, const Eigen::VectorXd& point_){
//     for (int i = 0; i < problem.numAgents(); i++){
//         robots_location.push_back(Eigen::Vector2d(point_(i*2), point_(i*2 + 1)));
//     }
// }

// MultiAgentDisk2DCollisionCheckerDecen::MultiAgentDisk2DCollisionCheckerDecen(const amp::MultiAgentProblem2D& problem_, const amp::MultiAgentPath2D& exisiting_path_)
// : problem(problem_), exisiting_path(exisiting_path_){
//     std::vector<std::pair<double,double>> bounds;
//     for(const amp::CircularAgentProperties& agent : problem.agent_properties) {
//         robots_radius.push_back(agent.radius);
//     }
//     currect_agent = exisiting_path.numAgents();
//     bounds.push_back({problem.x_min, problem.x_max});
//     bounds.push_back({problem.y_min, problem.y_max});
//     setBounds(bounds);

// }

// bool MultiAgentDisk2DCollisionCheckerDecen::isCollideWithTime(const Eigen::VectorXd& point, int time_step){
//     int num_of_sample = M_PI*pow(robots_radius[currect_agent]*1.2,2)*100;

//     // check if robot collide with enviroument
//     for (double theta = 0; theta< 2*M_PI; theta += 0.01){
//         Eigen::Vector2d point2check = point + Eigen::Vector2d(robots_radius[currect_agent]*cos(theta)*1.2, robots_radius[currect_agent]*sin(theta)*1.2);
//         for (int ob_idx = 0; ob_idx < problem.obstacles.size(); ++ob_idx) {
//             if (thisObstacle(ob_idx, point2check)){
//                 return true; // Point is inside this polygons
//             }
//         }
//     }

//     // check again if the robot collide with enviroument
//     for (int sample_idx = 0; sample_idx < num_of_sample; sample_idx++){
//         double theta = double(rand())/RAND_MAX * 2*M_PI;
//         double arm = double(rand())/RAND_MAX * robots_radius[currect_agent]*1.1;
//         Eigen::Vector2d point2check = point + Eigen::Vector2d(arm*cos(theta), arm*sin(theta));

//         for (int ob_idx = 0; ob_idx < problem.obstacles.size(); ++ob_idx) {
//             if (thisObstacle(ob_idx, point2check)){
//                 return true; // Point is inside this polygons
//             }
//         }
//     }
        
//     // check if robot collide with each other
//     for(int completed_agent_i = 0; completed_agent_i < exisiting_path.numAgents(); completed_agent_i ++){
//         if (exisiting_path.agent_paths[completed_agent_i].waypoints.size()-1 <= time_step){
//             double distance = (point - exisiting_path.agent_paths[completed_agent_i].waypoints.back()).norm();
//             if(distance <= (robots_radius[currect_agent] + robots_radius[completed_agent_i])*1.1){
//                 return true; //Robot Collide
//             }
//             continue;
//         }

//         // check if robot collide with other robot
//         Eigen::VectorXd one_2_two = exisiting_path.agent_paths[completed_agent_i].waypoints[time_step] - exisiting_path.agent_paths[completed_agent_i].waypoints[time_step-1];
//         double one_2_two_magnitude = one_2_two.norm();
//         int sec_num = 1;
//         while (one_2_two_magnitude/sec_num > GAPDECEN){
//             for(int i = 0; i < sec_num; i++){
//                 Eigen::VectorXd check_location = exisiting_path.agent_paths[completed_agent_i].waypoints[time_step-1] + one_2_two*(1+2*i)/(sec_num*2);
//                 double distance = (point - check_location).norm();
//                 if(distance <= (robots_radius[currect_agent] + robots_radius[completed_agent_i])*1.1){
//                     return true; //Robot Collide
//                 }
//             }
//             sec_num = sec_num * 2;
//         }
//     }
//     return false;
// }

// bool MultiAgentDisk2DCollisionCheckerDecen::isCollide2PWithTime(const Eigen::VectorXd& point1_, const Eigen::VectorXd& point2_, int time_step){
//     Eigen::VectorXd one_2_two = point2_ - point1_;
//     double distance = one_2_two.norm();
//     int sec_num = 1;
//     while(distance/sec_num > GAPDECEN){
//         for(int i = 0; i < sec_num; i++){
//             Eigen::VectorXd check_location = point1_ + one_2_two*(1+2*i)/(sec_num*2);
//             if (isCollideWithTime(check_location, time_step))
//             {
//                 return true;
//             }
//         }
//         sec_num = sec_num * 2;
//     }
//     return false;
// }

// bool MultiAgentDisk2DCollisionCheckerDecen::thisObstacle(int ob_idx_, const Eigen::Vector2d& point_){
//     const amp::Obstacle2D& obstacle = problem.obstacles[ob_idx_];
//     double cross_product;

//     int n = obstacle.verticesCCW().size();
//     for (int vertix_idx = 0; vertix_idx < n; ++vertix_idx) {
//         const Eigen::Vector2d& v1 = obstacle.verticesCCW()[vertix_idx];
//         const Eigen::Vector2d& v2 = obstacle.verticesCCW()[(vertix_idx + 1) % n];
//         Eigen::Vector2d edge = v1 - v2;
//         Eigen::Vector2d point_to_v1 = point_ - v1;
//         cross_product = edge(0) * point_to_v1(1) - edge(1) * point_to_v1(0);
//         if (cross_product > 0){
//             return false; // Point is outside of this obstacle
//         }
//     }
//     return true; // Point is inside this obstacle
// }

