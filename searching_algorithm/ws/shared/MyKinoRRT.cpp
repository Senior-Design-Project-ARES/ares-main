#include "MyKinoRRT.h"

void MyDynamicAgent::rungeKutta45(Eigen::VectorXd& state, const Eigen::VectorXd& control, double t) {
    double max_dy = 1.0;
    double dt = t*0.1;
    double t_pass = 0.0;
    while(true){
        if(t_pass + dt > t){
            dt = t - t_pass;
        }
        Eigen::VectorXd k1 = dynamic(state, control);
        Eigen::VectorXd k2 = dynamic(state + dt * (1.0/5.0) * k1, control);
        Eigen::VectorXd k3 = dynamic(state + dt * (3.0/40.0) * k1 + dt * (9.0/40.0) * k2, control);
        Eigen::VectorXd k4 = dynamic(state + dt * (44.0/45.0) * k1 - dt * (56.0/15.0) * k2 + dt * (32.0/9.0) * k3, control);
        Eigen::VectorXd k5 = dynamic(state + dt * (19372.0/6561.0) * k1 - dt * (25360.0/2187.0) * k2 + dt * (64448.0/6561.0) * k3 - dt * (212.0/729.0) * k4, control);
        Eigen::VectorXd k6 = dynamic(state + dt * (9017.0/3168.0) * k1 - dt * (355.0/33.0) * k2 + dt * (46732.0/5247.0) * k3 + dt * (49.0/176.0) * k4 - dt * (5103.0/18656.0) * k5, control);
        Eigen::VectorXd k7 = dynamic(state + dt * (35.0/384.0) * k1 + dt * (500.0/1113.0) * k3 + dt * (125.0/192.0) * k4 - dt * (2187.0/6784.0) * k5 + dt * (11.0/84.0) * k6, control);

        Eigen::VectorXd y_low = state + dt * ((35.0/384.0) * k1 + (500.0/1113.0) * k3 + (125.0/192.0) * k4 - (2187.0/6784.0) * k5 + (11.0/84.0) * k6);
        Eigen::VectorXd y_high = state + dt * ((5179.0/57600.0) * k1 + (7571.0/16695.0) * k3 + (393.0/640.0) * k4 - (92097.0/339200.0) * k5 + (187.0/2100.0) * k6 + (1.0/40.0) * k7);   
        max_dy = (y_high - y_low).cwiseAbs().maxCoeff();
        
        if(max_dy < EPSILON){
            t_pass += dt;
            state = y_high;
            if(t_pass >= t){
                return ;
            }
        }
        double scale = 0.9 * pow(EPSILON / (max_dy + 1e-10), 0.2);
        dt = dt*scale;
        if(dt < 1e-10){
            for(int i = 0 ; i < state.size(); i++) {
                state(i) = 1000000; // indicate failure
            }
            return;
        }

    }
}

Eigen::VectorXd MySingleIntegrator::dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    return control;
}

Eigen::VectorXd MyFirstOrderUnicycle::dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    Eigen::VectorXd dxdt(3);
    double theta = state(2);
    double v = control(0);
    double omega = control(1);
    dxdt(0) = v * cos(theta)*0.25;
    dxdt(1) = v * sin(theta)*0.25;
    dxdt(2) = omega;
    return dxdt;
}

Eigen::VectorXd MySecondOrderUnicycle::dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    Eigen::VectorXd dxdt(5);
    double theta = state(2);
    double v = state(3);
    double omega = state(4);
    double a = control(0);
    double alpha = control(1);
    dxdt(0) = v * cos(theta)*0.25;
    dxdt(1) = v * sin(theta)*0.25;
    dxdt(2) = omega;
    dxdt(3) = a;
    dxdt(4) = alpha;
    return dxdt;
}

Eigen::VectorXd MySimpleCar::dynamic(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    Eigen::VectorXd dxdt(5);
    double theta = state(2);
    double v = state(3);
    double phi = state(4);
    double a = control(0);
    double alpha = control(1);
    double L = agent_dim.length;
    // DEBUG(L);
    dxdt(0) = v * cos(theta);
    dxdt(1) = v * sin(theta);
    dxdt(2) = v / L * tan(phi);
    dxdt(3) = a;
    dxdt(4) = alpha;
    return dxdt;
}

// MyKinoRRT::MyKinoRRT(int max_iterations_, int control_sample_size_):
// max_iterations(max_iterations_), control_sample_size(control_sample_size_) {
// }

// double MyKinoRRT::distance(const amp::AgentType& agent_type, const Eigen::VectorXd& state1, const Eigen::VectorXd& state2) {
//     double angle_1, angle_2, angle_diff;
//     switch (agent_type)
//     {
//     case amp::AgentType::FirstOrderUnicycle:
//         // change to [-pi, pi]
//         angle_1 = unwrapAngle(state1(2));
//         angle_2 = unwrapAngle(state2(2));
//         angle_diff = fabs(angle_1 - angle_2);
//         if (angle_diff > M_PI){
//             angle_diff = 2*M_PI - angle_diff;
//         }
//         return sqrt(pow(state1(0) - state2(0), 2) + pow(state1(1) - state2(1), 2) + pow(angle_diff, 2));
//     case amp::AgentType::SecondOrderUnicycle:
//         // change to [-pi, pi]
//         angle_1 = unwrapAngle(state1(2));
//         angle_2 = unwrapAngle(state2(2));
//         angle_diff = fabs(angle_1 - angle_2);
//         if (angle_diff > M_PI){
//             angle_diff = 2*M_PI - angle_diff;
//         }
//         return sqrt(pow(state1(0) - state2(0), 2) + pow(state1(1) - state2(1), 2) + pow(angle_diff, 2) + pow(state1(3) - state2(3), 2) + pow(state1(4) - state2(4), 2));
//     case amp::AgentType::SimpleCar:
//         angle_1 = unwrapAngle(state1(2));
//         angle_2 = unwrapAngle(state2(2));
//         angle_diff = fabs(angle_1 - angle_2);
//         if (angle_diff > M_PI){
//             angle_diff = 2*M_PI - angle_diff;
//         }
//         return sqrt(pow(state1(0) - state2(0), 2) + pow(state1(1) - state2(1), 2) + pow(angle_diff, 2) + pow(state1(3) - state2(3), 2) + pow(state1(4) - state2(4), 2));
//     default:
//         return (state1 - state2).norm();
//     }
// }

// amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem_, amp::DynamicAgent& agent) {
//     amp::KinoPath path;

//     amp::KinodynamicProblem2D problem = problem_;
//     // if(problem.agent_type == amp::AgentType::SimpleCar && problem.agent_dim.length == 5){
//     //     DEBUG("here");
//     //     path = get_pre_plan_path();
//     //     amp::HW9::check(path, problem);
//     //     return path;
        
//     //     // return parking_path;
//     // }
//     // else{
//     //     return path;
//     // }

//     agent.agent_dim = problem.agent_dim;
//     Eigen::VectorXd state = problem.q_init;
//     Point2DCollisionChecker checker(problem);
//     bool reached_goal = false;

//     // prepare for polygon agent collision checking
//     if (!problem.isPointAgent) {
//         double angle = atan((problem.agent_dim.width/2)/(problem.agent_dim.length/2));
//         arm = Eigen::Vector2d(problem.agent_dim.length/2, problem.agent_dim.width/2)*1.1;
//         to_center = Eigen::Vector2d(problem.agent_dim.length/2, 0);
//         rot1 = Eigen::Rotation2D<double>(angle);
//         rot2 = Eigen::Rotation2D<double>(M_PI - angle);
//         rot3 = Eigen::Rotation2D<double>(-M_PI + angle);
//         rot4 = Eigen::Rotation2D<double>(-angle);
//         arm = rot4 * arm;
//     }

//     nodes[0] = problem.q_init;

//     for(int i = 0; i < max_iterations; i++) {
//         // if (i % 1000 == 0){
//         //     DEBUG("iterations: " << i);
//         // }
//         // if (problem.agent_type == amp::AgentType::SimpleCar){
//         //     DEBUG("iterations: " << i);
//         // }
//         Eigen::VectorXd rand_x(problem.q_init.size()); //random target state

//         if(rand()/RAND_MAX < 0.05) {
//             for(int i = 0 ; i < problem.q_goal.size(); i++) {
//                 rand_x(i) = (problem.q_goal[i].first + problem.q_goal[i].second)/2;
//             }
//         }
        

//         // sample random state
//         for (int i = 0; i < problem.q_init.size(); i++) {
//             rand_x(i) = problem.q_bounds[i].first + double(rand())/RAND_MAX*(problem.q_bounds[i].second - problem.q_bounds[i].first);
//         }

//         state = extendRRT(problem, checker, agent, rand_x);
//         if (state.isZero()){
//             continue; // failed to extend RRT
//         }

//         reached_goal = true;
//         for (int i = 0; i < problem.q_goal.size(); i++) {
//             if (state(i) < problem.q_goal[i].first || state(i) > problem.q_goal[i].second) {
//                 reached_goal = false;
//                 break; // not reached goal
//             }
//         }
//         if (reached_goal){
//             break;
//         }
//     }

//     if (!reached_goal){
//         state = problem.q_init;
//         path.valid = false;
//         path.waypoints.push_back(problem.q_init);
//         for (int i = 0; i < 10; i++) {
//             Eigen::VectorXd control = Eigen::VectorXd(problem.u_bounds.size());
//             for(int j = 0; j < problem.u_bounds.size(); j++) {
//                 control(j) = 0.1;
//             }


//             if(problem.agent_type == amp::AgentType::SimpleCar){
//                 Eigen::VectorXd temp_State(state.size()+1);
//                 for(int k = 0 ; k < state.size(); k++){
//                     temp_State(k) = state(k);
//                 }
//                 temp_State(temp_State.size() - 1) = problem.agent_dim.length; // L
//                 agent.propagate(temp_State, control, 1.0);
//                 state = temp_State.head(temp_State.size() - 1);
//             }
//             else {
//                 agent.propagate(state, control, 1.0);
//             }

//             // agent.propagate(state, control, 1.0);
//             path.waypoints.push_back(state);
//             path.controls.push_back(control);
//             path.durations.push_back(1.0);
//         }
//         // path.print();
//         return path;
//     }

//     // backtrack to get path
//     path.valid = true;
//     path.waypoints.push_back(nodes[nodes.size() - 1]);
//     amp::Node current_node = nodes.size() - 1;
//     while (current_node != 0) {
//         amp::Node parent_node = graphPtr->parents(current_node)[0];
//         std::vector<MyEdge> edges = graphPtr->outgoingEdges(parent_node);
//         std::vector<amp::Node> children = graphPtr->children(parent_node);
//         for (int i = 0; i < edges.size(); i++) {
//             if (children[i] == current_node) {
//                 path.controls.push_back(edges[i].control);
//                 path.durations.push_back(edges[i].dt);
//                 path.length += edges[i].length;
//                 break;
//             }
//         }
//         path.waypoints.push_back(nodes[parent_node]);
//         current_node = parent_node;
//     }
//     std::reverse(path.waypoints.begin(), path.waypoints.end());
//     std::reverse(path.controls.begin(), path.controls.end());
//     std::reverse(path.durations.begin(), path.durations.end());

//     amp::HW9::check(path, problem);
//     // amp::Visualizer::makeFigure(problem, path, false); // Set to 'true' to render animation
//     // amp::Visualizer::saveFigures(true, "hw9_figs");
//     return path;
// }

// Eigen::VectorXd MyKinoRRT::extendRRT(const amp::KinodynamicProblem2D& problem, Point2DCollisionChecker& checker, amp::DynamicAgent& agent, Eigen::VectorXd& rand_x){
//     //initialize control input
//     std::vector<Eigen::VectorXd> controls; 
//     Eigen::VectorXd state;

//     // get random duration
//     double dt = problem.dt_bounds.first + double(rand())/RAND_MAX*(problem.dt_bounds.second - problem.dt_bounds.first);
//     // find nearest node
//     double min_dist = std::numeric_limits<double>::max();
//     amp::Node nearest_node = -1;
//     for (const auto& node_pair : nodes) {
//         double dist = distance(problem.agent_type, node_pair.second, rand_x);
//         if (dist < min_dist) {
//             min_dist = dist;
//             state = node_pair.second;
//             nearest_node = node_pair.first;
//         }
//     }

//     // sample control inputs
//     for (int i = 0; i < control_sample_size; i++) {
//         Eigen::VectorXd control(problem.u_bounds.size());
//         for (int j = 0; j < problem.u_bounds.size(); j++) {
//             control(j) = problem.u_bounds[j].first + double(rand())/RAND_MAX*(problem.u_bounds[j].second - problem.u_bounds[j].first);
//         }
//         controls.push_back(control);
//     }

//     // propagate each control
//     Eigen::VectorXd end_state; // store the end state after applying control
//     double min_distance = std::numeric_limits<double>::max();
//     int nearest_control_idx = -1;
//     for (int i = 0; i < controls.size(); i++) {
//         Eigen::VectorXd new_state = state;
//         if(problem.agent_type == amp::AgentType::SimpleCar){
//             Eigen::VectorXd temp_State(new_state.size()+1);
//             for(int k = 0 ; k < new_state.size(); k++){
//                 temp_State(k) = new_state(k);
//             }
//             temp_State(temp_State.size() - 1) = problem.agent_dim.length; // L
//             agent.propagate(temp_State, controls[i], dt);
//             new_state = temp_State.head(temp_State.size() - 1);
//         }
//         else {
//             agent.propagate(new_state, controls[i], dt);
//         }
//         if (min_distance > distance(problem.agent_type, new_state, rand_x)) {
//             min_distance = distance(problem.agent_type, new_state, rand_x);
//             nearest_control_idx = i;
//             end_state = new_state;
//         }
//     }
//     if (end_state.size() > 2){
//         end_state(2) = unwrapAngle(end_state(2)); // change angle to [-pi, pi]
//         if(end_state(2) > M_PI || end_state(2) < -M_PI){
//             DEBUG("Angle wrapping failed!");
//         }
//     }

//     state = end_state;

//     // check if final state is in bounds
//     for(int i = 0; i < problem.q_init.size(); i++) {
//         if (i == 2){
//             state(i) = unwrapAngle(state(i)); // change angle to [-pi, pi]
//             continue; // ignore theta bound
//         }
//         if (state(i) > problem.q_bounds[i].second || state(i) < problem.q_bounds[i].first) {
//             return Eigen::VectorXd::Zero(problem.q_init.size());
//         }
//     }

//     // check if path collide with obstacles
//     if(checker.isCollide(state)){
//         return Eigen::VectorXd::Zero(problem.q_init.size()); // end point collide with obstacle
//     }
//     if(!problem.isPointAgent){
//         Eigen::Rotation2D<double> rot = Eigen::Rotation2D<double>(state(2));
//         Eigen::Vector2d corner1 = rot * to_center + rot * rot1 * arm + state.head<2>();
//         Eigen::Vector2d corner2 = rot * to_center + rot * rot2 * arm + state.head<2>();
//         Eigen::Vector2d corner3 = rot * to_center + rot * rot3 * arm + state.head<2>();
//         Eigen::Vector2d corner4 = rot * to_center + rot * rot4 * arm + state.head<2>();
//         // check if corner is out of bound
//         if(corner1(0) < problem.q_bounds[0].first || corner1(0) > problem.q_bounds[0].second ||
//            corner1(1) < problem.q_bounds[1].first || corner1(1) > problem.q_bounds[1].second ||
//            corner2(0) < problem.q_bounds[0].first || corner2(0) > problem.q_bounds[0].second ||
//            corner2(1) < problem.q_bounds[1].first || corner2(1) > problem.q_bounds[1].second ||
//            corner3(0) < problem.q_bounds[0].first || corner3(0) > problem.q_bounds[0].second ||
//            corner3(1) < problem.q_bounds[1].first || corner3(1) > problem.q_bounds[1].second ||
//            corner4(0) < problem.q_bounds[0].first || corner4(0) > problem.q_bounds[0].second ||
//            corner4(1) < problem.q_bounds[1].first || corner4(1) > problem.q_bounds[1].second){
//             return Eigen::VectorXd::Zero(problem.q_init.size()); // polygon agent out of bound
//         }
//         if(checker.isCollide(corner1) || checker.isCollide(corner2) || checker.isCollide(corner3) || checker.isCollide(corner4)){
//             return Eigen::VectorXd::Zero(problem.q_init.size()); // polygon agent collide with obstacle
//         }
//         // if(checker.isCollide2P(corner1, corner2) || checker.isCollide2P(corner3, corner4) || checker.isCollide2P(corner2, corner3) || checker.isCollide2P(corner4, corner1)){
//         //     return Eigen::VectorXd::Zero(problem.q_init.size()); // polygon agent collide with obstacle
//         // }
//     }

//     // check if path between two points collide with obstacles using propegate steps
//     state = nodes[nearest_node];
//     double dt_propagate = dt / 2;
//     std::vector<Eigen::VectorXd> intermediate_states;
//     intermediate_states.push_back(state);
//     bool collide = false;

//     while(distance(problem.agent_type, state, end_state) > 1e-3){
//         // if(distance(problem.agent_type, state, end_state) > 1e-2){
//         //     DEBUG(distance(problem.agent_type, state, end_state));
//         // }
//         Eigen::VectorXd old_state = state;
//         if(problem.agent_type == amp::AgentType::SimpleCar){
//             Eigen::VectorXd temp_State(state.size()+1);
//             for(int i = 0 ; i < state.size(); i++){
//                 temp_State(i) = state(i);
//             }
//             temp_State(temp_State.size() - 1) = problem.agent_dim.length; // L
//             agent.propagate(temp_State, controls[nearest_control_idx], dt_propagate);
//             state = temp_State.head(temp_State.size() - 1);
//         }
//         else {
//             agent.propagate(state, controls[nearest_control_idx], dt_propagate);
//         }
//         // agent.propagate(state, controls[nearest_control_idx], dt_propagate);
//         if(state.size() > 2){
//             state(2) = unwrapAngle(state(2)); // change angle to [-pi, pi]
//         }
//         double prop_dist = distance(problem.agent_type, old_state, state);

//         // check if inbounds
//         for(int i = 0; i < problem.q_init.size(); i++) {
//             if (i == 2){
//                 continue; // ignore theta bound
//             }
//             if (state(i) > problem.q_bounds[i].second || state(i) < problem.q_bounds[i].first) {
//                 return Eigen::VectorXd::Zero(problem.q_init.size());
//             }
//         }

//         if(checker.isCollide(state)){
//             state = old_state;
//             collide = true;
//             break; // path collide with obstacle
//         }
//         if(!problem.isPointAgent){
//             Eigen::Rotation2D<double> rot = Eigen::Rotation2D<double>(state(2));
//             Eigen::Vector2d corner1 = rot * to_center + rot * rot1 * arm + state.head<2>();
//             Eigen::Vector2d corner2 = rot * to_center + rot * rot2 * arm + state.head<2>();
//             Eigen::Vector2d corner3 = rot * to_center + rot * rot3 * arm + state.head<2>();
//             Eigen::Vector2d corner4 = rot * to_center + rot * rot4 * arm + state.head<2>();
//             // check if corner is out of bound
//             if(corner1(0) < problem.q_bounds[0].first || corner1(0) > problem.q_bounds[0].second ||
//             corner1(1) < problem.q_bounds[1].first || corner1(1) > problem.q_bounds[1].second ||
//             corner2(0) < problem.q_bounds[0].first || corner2(0) > problem.q_bounds[0].second ||
//             corner2(1) < problem.q_bounds[1].first || corner2(1) > problem.q_bounds[1].second ||
//             corner3(0) < problem.q_bounds[0].first || corner3(0) > problem.q_bounds[0].second ||
//             corner3(1) < problem.q_bounds[1].first || corner3(1) > problem.q_bounds[1].second ||
//             corner4(0) < problem.q_bounds[0].first || corner4(0) > problem.q_bounds[0].second ||
//             corner4(1) < problem.q_bounds[1].first || corner4(1) > problem.q_bounds[1].second){
//                 return Eigen::VectorXd::Zero(problem.q_init.size()); // polygon agent out of bound
//             }
//             if(checker.isCollide(corner1) || checker.isCollide(corner2) || checker.isCollide(corner3) || checker.isCollide(corner4)){
//                 return Eigen::VectorXd::Zero(problem.q_init.size()); // polygon agent collide with obstacle
//             }
//             // if(checker.isCollide2P(corner1, corner2) || checker.isCollide2P(corner3, corner4) || checker.isCollide2P(corner2, corner3) || checker.isCollide2P(corner4, corner1)){
//             //     collide = true;
//             //     break; // path collide with obstacle
//             // }
//         }

//         if(prop_dist > STEP_SIZE){
//             dt_propagate = dt_propagate / 2;
//             state = old_state;
//             continue; // reduce step size
//         }
//         intermediate_states.push_back(state);
//     }
//     if (collide){
//         return Eigen::VectorXd::Zero(problem.q_init.size());
//     }

//     if (state.size() > 2){
//         state(2) = unwrapAngle(state(2)); // change angle to [-pi, pi]
//     }

//     // add new node to graph
//     MyEdge edge;
//     edge.control = controls[nearest_control_idx];
//     edge.length = 0;
//     for (int i = 1; i < intermediate_states.size(); i++) {
//         Eigen::Vector2d first = intermediate_states[i - 1].head<2>();
//         Eigen::Vector2d second = intermediate_states[i].head<2>();
//         edge.length += (first - second).norm();
//     }
//     edge.dt = dt;
//     nodes[nodes.size()] = end_state;
//     graphPtr->connect(nearest_node, nodes.size() - 1, edge);
//     // DEBUG("Added node " << nodes.size() - 1 << " connected to " << nearest_node);

//     return end_state;
// }
