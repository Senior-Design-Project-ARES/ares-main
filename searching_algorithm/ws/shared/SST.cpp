#include "SST.h"

amp::Path SST::planND(Eigen::VectorXd init_, Eigen::VectorXd goal_) {
    int num_control_inputs = 2;
    int num_states = 3;
    // Add initial node as active to the graph
    nodes[0] = {init_, Eigen::VectorXd::Zero(num_control_inputs), 0.0};
    uint32_t node_count = 1;
    // Create new neighborhood at initial point
    neighborhoods.push_back(Neighborhood{init_, {0}});

    // Main SST loop
    for (int i = 0; i < iteration; i++) {
        // Sample random point in the space
        double x = amp::RNG::randd(/*problem.x_min, problem.x_max*/0.0, 10.0);
        double y = amp::RNG::randd(/*problem.y_min, problem.y_max*/0.0, 10.0);
        Eigen::Vector2d random_point = Eigen::Vector2d(x, y);

        // Find lowest cost node in delta_bn neighborhood
        // If none, use closest node
        amp::Node nearest_node = bestFirstSelection(random_point, nodes);

        // Extend with a random control input
        StateAndControl new_point = extendSST(nearest_node);
        // Check if new point is valid
        if (true /*envMap.isCollision(new_point) == false*/) {
            // Check if new point is in delta_s neighborhood of any existing neighborhood
            // if it is, check if it has lower cost than the existing nodes in that neighborhood
            bool in_neighborhood = false;
            bool found_better = false;
            for (auto& neighborhood : neighborhoods) {
                double dist = (new_point.state.head(2) - neighborhood.center).norm();
                if (dist < delta_s) {
                    in_neighborhood = true;
                    // Check if new point has lower cost than any node in the neighborhood
                    for (auto& node : neighborhood.nodes_in_neighborhood) {
                        double existing_cost = nodes[node].cost;
                        double new_cost = new_point.cost;
                        if (new_cost < existing_cost) {
                            // Add new point to graph
                            nodes[node_count] = new_point;
                            graphPtr->connect(nearest_node, node_count++, 1);
                            neighborhood.nodes_in_neighborhood.push_back(node_count);
                            // Make existing node inactive
                            nodes[node].active = false;
                            found_better = true;
                            break;
                        }
                    }
                    if (found_better) {
                        break;
                    }
                }
            }
            if (!in_neighborhood) {
                // Add new neighborhood
                neighborhoods.push_back(Neighborhood{new_point.state.head(2), {node_count}});
            }
        }
    }
    // Search for goal node in active nodes
        amp::Node goal_node = -1;
        amp::Node start_node = 0;
        for (const auto& pair : nodes) {
            const Eigen::VectorXd& state = pair.second.state;
            double dist_to_goal = (state.head(2) - goal_.head(2)).norm();
            if (dist_to_goal < delta_s) {
                goal_node = pair.first;
                break;
            }
        }

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
        path.waypoints.push_back(init_);
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

amp::Node SST::bestFirstSelection(const Eigen::Vector2d& point, const std::map<amp::Node, StateAndControl>& nodes){
    double closest_distance = -1;
    amp::Node closest_node;
    for (const auto& pair : nodes){
        double distance = (point - pair.second.state.head(2)).norm();
        if (distance < delta_bn){
            if (closest_distance == -1 || pair.second.cost < nodes.at(closest_node).cost){
                closest_distance = distance;
                closest_node = pair.first;
            }
        }
    }
    if (closest_distance != -1){
        return closest_node;
    }

    // likely more efficient to combine the two loops above
    // If no node in delta_bn neighborhood, return closest node
    closest_distance = -1;
    for (const auto& pair : nodes){
        double distance = (point - pair.second.state.head(2)).norm();
        if (closest_distance == -1 || distance < closest_distance){
            closest_distance = distance;
            closest_node = pair.first;
        }
    }
    return closest_node;
}

StateAndControl SST::extendSST(const amp::Node node){
    // Sample random control input
    Eigen::VectorXd control_input;
    control_input.resize(2);
    control_input(0) = amp::RNG::randd(-1.0, 1.0);
    control_input(1) = amp::RNG::randd(-1.0, 1.0);

    // use random time step and propagate dynamics
    double time_step = amp::RNG::randd(0.1, 0.5);
    Eigen::VectorXd new_state = nodes[node].state /* dynamics.step(node_positions[node].state, control_input, time_step)*/; // Placeholder for dynamics propagation
    double new_cost = nodes[node].cost + time_step; // Placeholder for cost calculation
    return {new_state, control_input, new_cost};
}