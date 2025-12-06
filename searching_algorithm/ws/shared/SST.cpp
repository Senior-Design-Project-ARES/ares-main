#include "SST.h"

amp::Path SST::planND(Eigen::VectorXd init_, Eigen::VectorXd goal_, amp::GridCSpace2D_T<int8_t> envMap) {
    printf("Starting SST planning...\n");
    // print initial and goal
    printf("Initial state: (%f, %f)\n", init_(0), init_(1));
    printf("Goal state: (%f, %f)\n", goal_(0), goal_(1));
    Eigen::VectorXd init(3);
    init << init_(0), init_(1), 0.0; // Assuming initial theta is 0
    Eigen::VectorXd goal(3);
    goal << goal_(0), goal_(1), 0.0; // Assuming goal theta is 0
    int num_control_inputs = 2;
    int num_states = 3;
    // Add initial node as active to the graph
    nodes[0] = {init, Eigen::VectorXd::Zero(num_control_inputs), 0.0};
    uint32_t node_count = 1;
    // Create new neighborhood at initial point
    neighborhoods.push_back(Neighborhood{init, {0}});

    printf("SST main loop...\n");
    // Main SST loop
    for (int i = 0; i < iteration; i++) {
        // printf("Iteration %d/%d\n", i + 1, iteration);
        // Sample random point in the space
        double x = amp::RNG::randd(envMap.x0Bounds().first, envMap.x0Bounds().second);
        double y = amp::RNG::randd(envMap.x1Bounds().first, envMap.x1Bounds().second);
        double theta = amp::RNG::randd(-M_PI, M_PI);
        Eigen::VectorXd random_point = Eigen::VectorXd(3);
        random_point << x, y, theta;

        // Find lowest cost node in delta_bn neighborhood
        // If none, use closest node
        // printf("Finding best first selection...\n");
        amp::Node nearest_node = bestFirstSelection(random_point, nodes);

        // Extend with a random control input
        // printf("Extending from nearest node %d...\n", nearest_node);
        StateAndControl new_point = extendSST(nearest_node);
        // printf("New point state: (%f, %f, %f) with cost %f\n", new_point.state(0), new_point.state(1), new_point.state(2), new_point.cost);
        // Check if new point is valid
        if (true /*collisionCheck(new_point, envMap)*/) {
            // Check if new point is in delta_s neighborhood of any existing neighborhood
            // if it is, check if it has lower cost than the existing nodes in that neighborhood
            bool in_neighborhood = false;
            bool found_better = false;
            for (auto& neighborhood : neighborhoods) {
                // printf("Checking neighborhood centered at (%f, %f)...\n", neighborhood.center(0), neighborhood.center(1));
                double dist = (new_point.state.head(3) - neighborhood.center).norm();
                // printf("Distance to neighborhood center: %f\n", dist);
                if (dist < delta_s) {
                    in_neighborhood = true;
                    // Check if new point has lower cost than any node in the neighborhood
                    for (auto& node : neighborhood.nodes_in_neighborhood) {
                        // printf("Comparing new point cost %f with existing node %d cost %f\n", new_point.cost, node, nodes[node].cost);
                        double existing_cost = nodes[node].cost;
                        double new_cost = new_point.cost;
                        if (new_cost < existing_cost) {
                            // Add new point to graph
                            nodes[node_count] = new_point;
                            graphPtr->connect(nearest_node, node_count, 1);
                            
                            // Make existing node inactive
                            // printf("Making node %d inactive\n", node);
                            nodes[node].active = false;
                            // printf("Added new node %d to neighborhood\n", node_count - 1);
                            found_better = true;
                            neighborhood.nodes_in_neighborhood.push_back(node_count);
                            node_count++;
                            break;
                        }
                    }
                    if (found_better) {
                        break;
                    }
                }
            }
            // printf("In neighborhood: %d, Found better: %d\n", in_neighborhood, found_better);
            if (!in_neighborhood) {
                // Add new neighborhood
                neighborhoods.push_back(Neighborhood{new_point.state.head(2), {node_count}});
                // Add new point to graph
                nodes[node_count] = new_point;
                graphPtr->connect(nearest_node, node_count++, 1);
                // printf("Added new neighborhood with node %d\n", node_count - 1);
            }
        }
    }
    printf("SST planning finished, extracting path...\n");
    // Search for goal node in active nodes
        amp::Node goal_node = -1;
        amp::Node start_node = 0;
        for (const auto& pair : nodes) {
            const Eigen::VectorXd& state = pair.second.state;
            double dist_to_goal = (state.head(2) - goal.head(2)).norm();
            if (dist_to_goal < delta_s) {
                goal_node = pair.first;
                break;
            }
        }
    if (goal_node == -1) {
        printf("No goal node found within delta_s of goal\n");
        amp::Path empty_path;
        empty_path.valid = false;
        return empty_path;
    }
    printf("Goal node found: %d\n", goal_node);
    // Extract path from start to goal using tree structure
        amp::KinoPath path;
        amp::Node current_node = goal_node; // goal node
        int loop_counter = 0;
        // printf("Extracting path from goal node %d to start node %d\n", goal_node, start_node);
        while (current_node != start_node) {
            loop_counter++;
            if (loop_counter > 10000) {
                printf("Error: Exceeded maximum iterations while extracting path\n");
                break;
            }
            path.waypoints.push_back(nodes[current_node].state);
            path.controls.push_back(nodes[current_node].control);
            path.durations.push_back(0.1);
            std::vector<amp::Node> parents = graphPtr->parents(current_node);
            if (parents.size() != 1) {
                printf("Error: SST tree structure invalid, size %ld\n", parents.size());
                break;
            }
            current_node = parents.front(); // Move to parent
            // printf("Current node in path extraction: %d, pos: (%f, %f)\n", current_node, node_positions[current_node].x(), node_positions[current_node].y());
        }
        path.waypoints.push_back(init);
        std::reverse(path.waypoints.begin(), path.waypoints.end());
        double path_length = 0.0;
        double path_duration = 0.0;
        for (size_t i = 1; i < path.waypoints.size(); ++i) {
            path_length += (path.waypoints[i] - path.waypoints[i - 1]).norm();
            path_duration += 0.1;
        }
        printf("Path found in SST with length: %f and duration: %f\n", path_length, path_duration);

        path.valid = true;
        return path;
}

amp::Node SST::bestFirstSelection(const Eigen::VectorXd& point, const std::map<amp::Node, StateAndControl>& nodes){
    double closest_distance = -1;
    amp::Node closest_node = 0;
    for (const auto& pair : nodes){
        if (!pair.second.active){
            continue; // only consider active nodes
        }
        // printf("Evaluating node %d...\n", pair.first);
        if(pair.second.state.size() < 3){
            printf("Node %d state size invalid: %ld\n", pair.first, pair.second.state.size());
            // continue; // skip invalid states
        }
        double distance = (point - pair.second.state.head(3)).norm();
        // printf("Checking node %d with distance %f\n", pair.first, distance);
        if (distance < delta_bn){
            // printf("cost closed node %d with cost %f\n", pair.first, pair.second.cost);
            if (closest_distance == -1 || pair.second.cost < nodes.at(closest_node).cost){
                closest_distance = distance;
                closest_node = pair.first;
                // printf("New best node in neighborhood: %d with cost %f\n", closest_node, pair.second.cost);
            }
        }
    }
    // printf("Best first selection found node %d with distance %f\n", closest_node, closest_distance);
    if (closest_distance != -1){
        return closest_node;
    }
    // printf("No node found in delta_bn neighborhood, searching for closest node...\n");
    // likely more efficient to combine the two loops above
    // If no node in delta_bn neighborhood, return closest node
    closest_distance = -1;
    for (const auto& pair : nodes){
        if (!pair.second.active){
            continue; // only consider active nodes
        }
        double distance = (point - pair.second.state.head(3)).norm();
        if (closest_distance == -1 || distance < closest_distance){
            closest_distance = distance;
            closest_node = pair.first;
        }
    }
    // printf("Closest node found: %d with distance %f\n", closest_node, closest_distance);
    return closest_node;
}

StateAndControl SST::extendSST(const amp::Node node){
    // Sample random control input
    Eigen::VectorXd control_input(2);
    control_input(0) = amp::RNG::randd(-1.0, 1.0);
    control_input(1) = amp::RNG::randd(-1.0, 1.0);

    // use random time step and propagate dynamics
    // double time_step = amp::RNG::randd(0.1, 0.5);
    double time_step = 0.5;
    // printf("Propagating from node %d with control (%f, %f) for time %f\n", node, control_input(0), control_input(1), time_step);
    Eigen::VectorXd temp_state = nodes[node].state;
    // printf("Current state before propagation: (%f, %f, %f)\n", temp_state(0), temp_state(1), temp_state(2));
    agent.propagate(temp_state, control_input, time_step); // Placeholder for dynamics propagation
    // printf("New state after propagation: (%f, %f, %f)\n", temp_state(0), temp_state(1), temp_state(2));
    double new_cost = nodes[node].cost + time_step;// + (temp_state.head(2) - nodes[node].state.head(2)).norm(); // Placeholder for cost calculation
    // printf("New cost after propagation: %f\n", new_cost);
    return {temp_state, control_input, new_cost};
}