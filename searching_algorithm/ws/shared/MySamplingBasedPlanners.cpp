# include "MySamplingBasedPlanners.h"

// MyGenericPRM
// MyGenericPRM::MyGenericPRM(int num_nodes, int k_neighbors_, double radius_, bool smooth_)
// : num_nodes(num_nodes), k_neighbors(k_neighbors_), radius(radius_), smooth(smooth_) {}

// amp::Path MyGenericPRM::planND(Eigen::VectorXd init_, Eigen::VectorXd goal_, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_){
//     nodes[nodes.size()] = init_;
//     nodes[nodes.size()] = goal_;

//     connectClosest(nodes.size()-2, collision_checker_);
//     connectClosest(nodes.size()-1, collision_checker_);

//     amp::ShortestPathProblem problem;
//     problem.graph = graphPtr;
//     problem.goal_node = nodes.size()-1;
//     problem.init_node = nodes.size()-2;
//     MyAStarAlgo algo;
//     amp::SearchHeuristic heuristic;
//     MyAStarAlgo::GraphSearchResult result = algo.search(problem, heuristic);

//     amp::Path path;

//     if (result.success){
//         path.waypoints.push_back(init_);
//         auto it = result.node_path.begin();
//         for (int i = 1; i < result.node_path.size()-1; i++){
//             std::advance(it, 1);
//             path.waypoints.push_back(nodes[*it]);
//         }
//         path.waypoints.push_back(goal_);

//         if(smooth){
//             return smoothPath(path, collision_checker_);
//         }
        
//         return path;
//     }

//     return path;
// }

// void MyGenericPRM::createGraph(BaseCollisionChecker<Eigen::VectorXd>& collision_checker_){
//     graphPtr->clear();
//     nodes.clear();

//     int current_num_nodes = 0;
    
//     // create nodes
//     while (current_num_nodes < num_nodes){
//         Eigen::VectorXd node_location;
//         node_location.resize(collision_checker_.getBounds().size());
//         for (int dimension = 0; dimension < collision_checker_.getBounds().size(); dimension++){
//             double upper_bound = collision_checker_.getBounds().at(dimension).first;
//             double lower_bound = collision_checker_.getBounds().at(dimension).second;
//             node_location(dimension) = lower_bound + double(rand())*(upper_bound-lower_bound)/RAND_MAX;
//         }

//         if (!collision_checker_.isCollide(node_location)){
//             nodes.insert({current_num_nodes, node_location});
//             node_location.setZero();
//             current_num_nodes++;
//         }
//     }

//     // create edge
//     if(k_neighbors != 0)
//     {    
//         for (int node_idx = 0; node_idx < num_nodes; node_idx++)
//         {
//             std::unordered_map<amp::Node, double> closest_nodes = findClosest(nodes[node_idx], k_neighbors);
//             for(const auto& pair : closest_nodes){
//                 if (!collision_checker_.isCollide2P(nodes[node_idx], nodes[pair.first])){
//                     graphPtr->connect(node_idx, pair.first, pair.second);
                    
//                 }
//             }
//         }
//         return;
//     }
//     for (int node_idx = 0; node_idx < num_nodes; node_idx++)
//     {
//         for(const auto& pair : nodes){
//             if(pair.first == node_idx){
//                 continue;
//             }
//             double distance = (nodes[node_idx] - pair.second).norm();
//             if (distance < radius && !collision_checker_.isCollide2P(nodes[node_idx], pair.second)){
//                 graphPtr->connect(node_idx, pair.first, distance);
//             }
//         }
//     }
// }

// std::unordered_map<amp::Node, double> MyGenericPRM::findClosest(Eigen::VectorXd location, int num_of_closest){
//     std::unordered_map<amp::Node, double> closest_nodes;
//     int farthest_node = -1;
//     double farthest_distance = -1;

//     for (int node_idx_check = 0; node_idx_check < num_nodes; node_idx_check++){
//         double distance = (location - nodes[node_idx_check]).norm();

//         if(distance < pow(10, -6)){
//             continue;
//         }

//         if (closest_nodes.size() < num_of_closest){
//             closest_nodes[node_idx_check] = distance;

//             if (farthest_distance < distance || farthest_distance == -1){
//                 farthest_distance = distance;
//                 farthest_node = node_idx_check;
//             }

//             continue;
//         }
        
//         if (distance < farthest_distance){
//             closest_nodes.erase(farthest_node);
//             closest_nodes[node_idx_check] = distance;

//             int temp_farthest_node = -1;
//             double temp_farthest_distance = -1;
//             for (const auto& pair : closest_nodes){
//                 if(temp_farthest_distance < pair.second || temp_farthest_distance == -1){
//                     temp_farthest_distance = pair.second;
//                     temp_farthest_node = pair.first;
//                 }
//             }
//             farthest_node = temp_farthest_node;
//             farthest_distance = temp_farthest_distance;
//         }
//     }

//     return closest_nodes;
// }

// void MyGenericPRM::connectClosest(amp::Node node_idx, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_){
//     std::unordered_map<int, double> map;
//     std::vector<int> arr;
//     for(const auto& pair : getNodes()){
//         if(pair.first == node_idx){
//             continue;
//         }
//         map[pair.first] = (nodes[node_idx] - pair.second).norm();
//         arr.push_back(pair.first);
//     }

//     QuickSort<double> qs;
//     qs.run(arr, map, 0, arr.size()-1);

//     for (int i = 0; i < nodes.size(); i++){
//         if(arr[i] == node_idx){
//             continue;
//         }
//         if(!collision_checker_.isCollide2P(nodes[node_idx],nodes[arr[i]])){
//             double distance = (nodes[node_idx] - nodes[arr[i]]).norm();
//             graphPtr->connect(node_idx, arr[i], distance);
//             graphPtr->connect(arr[i], node_idx, distance);
//             return;
//         }
//     }
// }

// amp::Path MyGenericPRM::smoothPath(amp::Path path, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_){
//     int n_point = path.waypoints.size();
//     int n = 0;
//     while(n <= n_point/5 && n_point > 2){
//         int i = rand()%(n_point - 2) + 1;
//         if(!collision_checker_.isCollide2P(path.waypoints[i-1], path.waypoints[i+1])){
//             path.waypoints.erase(path.waypoints.begin()+i);
//             n_point--;
//             n = 0;
//             continue;
//         }
//         n++;
//     }
//     return path;
// }

// // MyPRM
// MyPRM::MyPRM(int num_nodes_, int k_neighbors_, double radius_, bool smooth_) 
// : MyGenericPRM(num_nodes_, k_neighbors_, radius_, smooth_) {}

// amp::Path2D MyPRM::plan(const amp::Problem2D& problem_) {
//     Point2DCollisionChecker collision_checker(problem_);
//     createGraph(collision_checker);
//     amp::Path ori_path = planND(problem_.q_init, problem_.q_goal, collision_checker);
//     amp::Path2D path;
//     path.waypoints = ori_path.getWaypoints2D();
//     // path.waypoints.push_back(problem_.q_init);
//     // path.waypoints.push_back(problem_.q_goal);
//     return path;
// }

// std::map<amp::Node, Eigen::Vector2d> MyPRM::getNodes2D(){
//     std::map<amp::Node, Eigen::Vector2d> nodes_2d;
//     for(const auto& pair : getNodes()){
//         nodes_2d.insert({pair.first, Eigen::Vector2d(pair.second(0), pair.second(1))});
//     }
//     return nodes_2d;
// }

// MyGenericRRT
amp::Path MyGenericRRT::planND(Eigen::VectorXd init_, Eigen::VectorXd goal_, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_){
    graphPtr->clear();
    nodes.clear();
    nodes[0] = init_;
    bool success = false;

    int temp = 0;
    for (int num_it = 0; num_it < iteration; num_it++){
        int step_num = 0;
        if(double(rand())/RAND_MAX < bias){
            Eigen::VectorXd new_point;
            do{
                new_point = extendRRT(goal_, collision_checker_);
                if(checkDistance(new_point, pow(10,-6))){
                    break;
                }
                step_num++;
            }while(!checkDistance(goal_ - new_point, pow(10,-6)));
            
            if(checkDistance(goal_ - new_point, pow(10,-5))){
                success = true;
                break;
            }

            continue;
        }

        Eigen::VectorXd rand_point = generatePoint(collision_checker_.getBounds());
        Eigen::VectorXd new_point = extendRRT(rand_point, collision_checker_);
        // if (new_point.isZero() || step_num == 0){
        //     num_it--;
        // }
        // if (num_it != temp){
        //     LOG(num_it);
        // }
        temp = num_it;
    }
    // LOG(temp);

    amp::Path path;
    path.valid = false;
    if(success){
        path.valid = true;
        path.waypoints.push_back(goal_);
        amp::Node current_node = (graphPtr->parents(nodes.size()-1))[0];
        while(current_node != 0){
            path.waypoints.insert(path.waypoints.begin(), nodes[current_node]);
            current_node = (graphPtr->parents(current_node))[0];
        }
        path.waypoints.insert(path.waypoints.begin(), init_);
    }
    return path;
}

amp::Path MyGenericRRT::planNDDecen(Eigen::VectorXd init_, Eigen::VectorXd goal_, MultiAgentDisk2DCollisionCheckerDecen& collision_checker_){
    graphPtr->clear();
    nodes.clear();
    nodes[0] = init_;
    bool success = false;

    int temp = 0;
    for (int num_it = 0; num_it < iteration; num_it++){
        int step_num = 0;
        if(double(rand())/RAND_MAX < bias){
            Eigen::VectorXd new_point;
            do{
                new_point = extendRRTDecen(goal_, collision_checker_);
                if(checkDistance(new_point, pow(10,-6))){
                    break;
                }
                step_num++;
            }while(!checkDistance(goal_ - new_point, pow(10,-6)));
            
            if(checkDistance(goal_ - new_point, pow(10,-5))){
                success = true;
                break;
            }

            continue;
        }

        Eigen::VectorXd rand_point = generatePoint(collision_checker_.getBounds());
        Eigen::VectorXd new_point = extendRRTDecen(rand_point, collision_checker_);
        // if (new_point.isZero() || step_num == 0){
        //     num_it--;
        // }
        // if (num_it != temp){
        //     LOG(num_it);
        // }
        temp = num_it;
    }
    // DEBUG(temp);

    amp::Path path;
    if(success){
        path.waypoints.push_back(goal_);
        amp::Node current_node = (graphPtr->parents(nodes.size()-1))[0];
        while(current_node != 0){
            path.waypoints.insert(path.waypoints.begin(), nodes[current_node]);
            current_node = (graphPtr->parents(current_node))[0];
        }
        path.waypoints.insert(path.waypoints.begin(), init_);
    }
    return path;
}


Eigen::VectorXd MyGenericRRT::generatePoint(const std::vector<std::pair<double, double>>& bounds){
    Eigen::VectorXd point;
    point.resize(bounds.size());
    for(int i = 0; i < bounds.size(); i++){
        double lower = bounds[i].first;
        double upper = bounds[i].second;
        point(i) = lower + (upper - lower)*double(rand())/RAND_MAX;
    }
    return point;
}

amp::Node MyGenericRRT::closestPoint(const Eigen::VectorXd& point){
    double closest_distance = -1;
    amp::Node closest_node;
    for (const auto& pair : getNodes()){
        if (closest_distance > magnitude(point-pair.second) || closest_distance == -1){
            closest_distance = magnitude(point-pair.second);
            closest_node = pair.first;
        }
        // if (closest_distance > (point-pair.second).norm() || closest_distance == -1){
        //     closest_distance = (point-pair.second).norm();
        //     closest_node = pair.first;
        // }
    }
    return closest_node;
}

Eigen::VectorXd MyGenericRRT::extendRRT(const Eigen::VectorXd& point, BaseCollisionChecker<Eigen::VectorXd>& collision_checker_){
    amp::Node closest_node = closestPoint(point);
    Eigen::VectorXd one_2_two = point - nodes[closest_node];
    Eigen::VectorXd step = one_2_two;
    
    if(checkDistance(one_2_two, pow(10,-6))){
        return step;
    }

    for(int i = 0 ; i < one_2_two.size()/2; i++){
        Eigen::Vector2d temp = {one_2_two(2*i), one_2_two(2*i+1)};
        if(temp.norm() < step_size){
            continue;
        }
        temp.normalize();
        step(2*i) = temp(0)*step_size;
        step(2*i+1) = temp(1)*step_size;
    }

    if(!collision_checker_.isCollide2P(nodes[closest_node], nodes[closest_node]+step)){
        nodes[nodes.size()] = nodes[closest_node]+step;
        graphPtr->connect(closest_node, nodes.size()-1, step.norm());
        return nodes[closest_node]+step;
    }
    step.setZero();
    return step;
}

Eigen::VectorXd MyGenericRRT::extendRRTDecen(const Eigen::VectorXd& point, MultiAgentDisk2DCollisionCheckerDecen& collision_checker_){
    amp::Node closest_node = closestPoint(point);
    Eigen::VectorXd one_2_two = point - nodes[closest_node];
    Eigen::VectorXd step = one_2_two;
    
    if(checkDistance(one_2_two, pow(10,-6))){
        return step;
    }

    for(int i = 0 ; i < one_2_two.size()/2; i++){
        Eigen::Vector2d temp = {one_2_two(2*i), one_2_two(2*i+1)};
        if(temp.norm() < step_size){
            continue;
        }
        temp.normalize();
        step(2*i) = temp(0)*step_size;
        step(2*i+1) = temp(1)*step_size;
    }

    int time_step = 1;
    if (closest_node != 0){
        amp::Node parent_nodes = graphPtr->parents(closest_node)[0];
        time_step++;
        while (parent_nodes != 0){
            parent_nodes = graphPtr->parents(parent_nodes)[0];
            time_step++;
        }
    }
    
    if(!collision_checker_.isCollide2PWithTime(nodes[closest_node], nodes[closest_node]+step, time_step)){
        nodes[nodes.size()] = nodes[closest_node]+step;
        graphPtr->connect(closest_node, nodes.size()-1, step.norm());
        return nodes[closest_node]+step;
    }
    step.setZero();
    return step;
}

bool MyGenericRRT::checkDistance(Eigen::VectorXd one_2_two, double requirement){
    std::vector<Eigen::Vector2d> direction;

    for (int i = 0; i < one_2_two.size()/2; i++){
        direction.push_back(Eigen::Vector2d(one_2_two(2*i), one_2_two(2*i+1)));
    }

    for(int i; i < direction.size(); i++){
        if(direction[i].norm()>requirement){
            return false;
        }
    }
    return true;
}

double MyGenericRRT::magnitude(Eigen::VectorXd vec){
    double magnitude = 0.0;
    for(int i = 0; i < vec.size()/2; i++){
        magnitude += pow(vec(2*i)*vec(2*i) + vec(2*i+1)*vec(2*i+1),0.5);
    }
    // DEBUG(magnitude);
    return magnitude;
}

// MyRRT
amp::Path2D MyRRT::plan(const amp::Problem2D& problem_) {
    amp::Path2D path;
    Point2DCollisionChecker collision_checker(problem_);
    amp::Path ori_path = planND(problem_.q_init, problem_.q_goal, collision_checker);
    path.waypoints = ori_path.getWaypoints2D();
    // path.waypoints.push_back(problem_.q_init);
    // path.waypoints.push_back(problem_.q_goal);
    return path;
}

std::map<amp::Node, Eigen::Vector2d> MyRRT::getNodes2D(){
    std::map<amp::Node, Eigen::Vector2d> nodes_2d;
    for(const auto& pair : getNodes()){
        nodes_2d.insert({pair.first, Eigen::Vector2d(pair.second(0), pair.second(1))});
    }
    return nodes_2d;
}
