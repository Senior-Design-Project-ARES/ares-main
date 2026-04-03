/// \file
/// \brief This node causes the robot to move autonomously via Frontier Exploration

#include "searching_and_planning/core/frontierSearching.h"

FrontExpl::FrontExpl(int map_width, int map_height, double resolution, const Eigen::Vector2d& origin, const std::vector<int8_t>& FE0_map, const searching_and_planning::Config& config)
    :config(config),
    map_width(map_width), 
    map_height(map_height), 
    FE0_map(FE0_map), 
    resolution(resolution), 
    origin(origin){}

void FrontExpl::neighborhood(int cell)
{
    // Clear any previous values in the vectors
    neighbor0_index.clear();
    neighbor0_value.clear();

    // Store neighboring values and indexes
    neighbor0_index.push_back(cell - map_width - 1);
    neighbor0_index.push_back(cell - map_width);
    neighbor0_index.push_back(cell - map_width + 1);
    neighbor0_index.push_back(cell-1);
    neighbor0_index.push_back(cell+1);
    neighbor0_index.push_back(cell + map_width - 1);
    neighbor0_index.push_back(cell + map_width);
    neighbor0_index.push_back(cell + map_width + 1);

    sort( neighbor0_index.begin(), neighbor0_index.end() );
    neighbor0_index.erase( unique( neighbor0_index.begin(), neighbor0_index.end() ), neighbor0_index.end() );
}

void FrontExpl::find_all_edges()
{
    // std::cout << "Finding all the edges" << std::endl;
    
    // Starting one row up and on space in on the map so there are no indexing issues
    for (int x = map_width + 1; x < (map_width * map_height) - map_width - 1; x++)
    {
        // Ignore the left and right edges of the map so there are no indexing issues
        if (x%map_width == 0 || x%map_width == map_width - 1)
        {
            continue;
        }

        // For all cells in the map, check if a cell is unknown
        if (FE0_map.at(x) == -1)
        {
            // If there is an unknown cell, then check neighboring cells to find a potential frontier edge (free cell)
            neighborhood(x);

            for(size_t i = 0; i < neighbor0_index.size(); i++) // For all neighboring cells
            {
                if ((i==1||i==3||i==4||i==6) && FE0_map.at(neighbor0_index.at(i)) == 0)
                {
                    // If the top, botoom left, or right is free, store it in the edges vector
                    // FE0_map.at(neighbor0_index.at(i)) = 10; // Visualize the frontier edge cells
                    edge0_vec.push_back(neighbor0_index.at(i));

                }
            }

        }
    } 

}

bool FrontExpl::check_edges(int curr_cell, int next_cell)
{
    if (curr_cell == next_cell)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void FrontExpl::find_regions()
{
    // std::cout << "Finding regions" << std::endl;

    std::vector<bool> visited(FE0_map.size(), false);


    for (size_t q = 0; q < edge0_vec.size() - 1; q++)
    {
        // For each frontier edge, check that the next value is unique and not a repeat
        unique_flag = check_edges(edge0_vec.at(q), edge0_vec.at(q+1));

        if (unique_flag == true && visited[edge0_vec.at(q)] == false)
        {
            // If we have an original frontier edge, check the neighboring cells
            neighborhood(edge0_vec.at(q));
            std::vector<FrontNode> frontier_group;
            bool more_than_two_neighbors = false;
            frontier_group.push_back(FrontNode(edge0_vec.at(q)));
            bool loop = false;

            // Check if there is more than 2 frontier neighbors
            for(size_t i = 0; i < neighbor0_index.size(); i++)
            {
                for (size_t j = 0; j < edge0_vec.size(); j++)
                {
                    if (neighbor0_index.at(i) == edge0_vec.at(j))
                    {
                        if (frontier_group[0].setFrontierNeighbors(neighbor0_index.at(i)) == false){
                            more_than_two_neighbors = true;
                            break;
                        }
                    }
                }
            }
            if (more_than_two_neighbors){
                continue;
            }

            // If less than 2 neighbors, continue adding to the region
            visited[edge0_vec.at(q)] = true;
            int previous_index = edge0_vec.at(q);
            int index_to_check = frontier_group[0].getFrontierNeighbors().first;
            // int previous_vector_index = 0;
            bool checking_second_direction = false;
            
            // Continue adding to the region until there are no more neighbors
            while(index_to_check != 0){
                // Mark the cell as visited
                visited[index_to_check] = true;

                // Add the cell to the region
                frontier_group.push_back(FrontNode(index_to_check));

                // Set the first neighbor to be where we came from
                frontier_group.back().setFrontierNeighbors(previous_index);

                // Get index of neighborhood of the cell
                neighborhood(index_to_check);

                // Check all the neighbors of the cell
                for(size_t i = 0; i < neighbor0_index.size(); i++)
                {
                    // If the neighbour is where we came from or it is within other region, skip it
                    if (neighbor0_index.at(i) == previous_index){
                        continue;
                    }

                    if (neighbor0_index.at(i) == edge0_vec.at(q)){
                        loop = true;
                        break;
                    }

                    if (visited[neighbor0_index.at(i)] == true){
                        continue;
                    }

                    // Go through all the frontier edges
                    for (size_t j = 0; j < edge0_vec.size(); j++)
                    {
                        // Check if the neighbor is a frontier edge
                        if (neighbor0_index.at(i) == edge0_vec.at(j))
                        {
                            previous_index = index_to_check; // consider the current cell as where we came from
                            
                            // If it is a frontier edge, set it as the next cell to check
                            // Check if there is more than 2 frontier neighbors for this frontier edge
                            // If there are more than 2 neighbors, consider this direction done
                            // switch direction if haven't already
                            // If already switched direction, stop
                            if (frontier_group.back().setFrontierNeighbors(neighbor0_index.at(i)) == false){
                                // Remove the last added cell since it has more than 2 neighbors
                                frontier_group.back().setFrontierNeighbors(2, 0);

                                // stop, and change diretion if haven't already
                                if (!checking_second_direction){
                                    checking_second_direction = true;
                                    previous_index = frontier_group[0].getIndex();
                                    index_to_check = frontier_group[0].getFrontierNeighbors().second;
                                    break;
                                }

                                // If we already changed direction, stop
                                index_to_check = 0;
                                break;

                                
                            }
                            index_to_check = neighbor0_index.at(i);
                        }
                    }
                }
                
                if (frontier_group.back().getFrontierNeighbors().second == 0){
                    // If no second neighbor, and we havent switched direction yet, switch direction
                    if (!checking_second_direction){
                        checking_second_direction = true;
                        previous_index = frontier_group[0].getIndex();
                        index_to_check = frontier_group[0].getFrontierNeighbors().second;
                        continue;
                    }
                    // If we already changed direction, stop
                    index_to_check = 0;
                }
            }

            frontier_regions.push_back(std::pair<std::vector<FrontNode>, bool>(frontier_group, loop));
        }

    }

}

void FrontExpl::find_centroids(){
    // std::cout << "Finding centroids" << std::endl;

    // For each region, find the centroid
    for (std::pair<std::vector<FrontNode>, bool> frountier_pair : frontier_regions)
    {
        double length = frountier_pair.first.size() * resolution;

        if (length < config.absolute_min_region_length) {
            // std::cout << "Region too small, skipping" << std::endl;
            continue;
        }

        int number_of_groups;
        if (length < 1.4)
        {
            number_of_groups = 1;
        }
        else
        {
            number_of_groups = floor(length / config.min_region_length);
        }
        int cells_per_group = floor(frountier_pair.first.size() / number_of_groups);
        int left_over = frountier_pair.first.size() - (cells_per_group * number_of_groups);
        // if it is a loop, just start at random place, at it is already in sequence
        if (frountier_pair.second == true){
            for (size_t current_start = 0; current_start < frountier_pair.first.size(); current_start += cells_per_group)
            {
                if (left_over > 0){
                    left_over -= 1;
                    current_start += 1;
                }
                int centroid_index = current_start + floor(cells_per_group / 2);
                centroids0.push_back(frountier_pair.first.at(centroid_index).getIndex());
                points_in_regions.push_back(cells_per_group);
            }
            continue;
        }

        int end_node_index = 0;
        std::map<int, int> global_to_local_map;
        // find the grid that touch a "wall"
        for (size_t node_index = 0; node_index < frountier_pair.first.size(); node_index++)
        {
            global_to_local_map[frountier_pair.first[node_index].getIndex()] = node_index;
            if(frountier_pair.first[node_index].getFrontierNeighbors().second == 0){
                end_node_index = node_index;
                break;
            }
        }

        if (end_node_index != 0)
        {
            std::rotate(frountier_pair.first.begin(),frountier_pair.first.begin()+end_node_index+1, frountier_pair.first.end());
        }
        // DEBUG("size: " << frountier_pair.first.size());
        for (size_t current_start = 0; current_start < frountier_pair.first.size(); current_start += cells_per_group)
        {
            size_t centroid_index = current_start + floor(cells_per_group / 2);
            if (left_over > 0){
                left_over -= 1;
                current_start += 1;
            }
            // DEBUG(centroid_index);
            centroids0.push_back(frountier_pair.first.at(centroid_index).getIndex());
            // points_in_regions.push_back(cells_per_group);
            points_in_regions.push_back(frountier_pair.first.size()/number_of_groups);
        }

        // int node_index = frountier_pair.first[end_node_index].getFrontierNeighbors().first;
        // int previous_index = end_node_index;
        // int cell_in_this_group = 2;
        // for (int cell_num = 1; cell_num < frountier_pair.first.size()-1; cell_num++)
        // {
        //     if(cell_in_this_group == floor(cells_per_group/2)+1)
        //     {
        //         centroids0.push_back(frountier_pair.first.at(global_to_local_map[node_index]).getIndex());
        //     }

        //     if(left_over>0 && cell_in_this_group == cells_per_group +1)
        //     {
        //         cell_in_this_group = 0;
        //         left_over--;
        //     }

        //     if(left_over==0 && cell_in_this_group == cells_per_group)
        //     {
        //         cell_in_this_group = 0;
        //     }

        //     if(frountier_pair.first.at(global_to_local_map[node_index]).getFrontierNeighbors().first != previous_index)
        //     {
        //         previous_index = node_index;
        //         node_index = frountier_pair.first.at(global_to_local_map[node_index]).getFrontierNeighbors().first;
        //     }
        //     else
        //     {
        //         previous_index = node_index;
        //         node_index = frountier_pair.first.at(global_to_local_map[node_index]).getFrontierNeighbors().second;
        //     }
        //     cell_in_this_group++;
        // }

    }
}

void FrontExpl::centroid_index_to_point()
{
    for (size_t t = 0; t < centroids0.size(); t++)
    {
        // For all the centorid cells, find the x and y coordinates in the map frame
        point(0) = (centroids0.at(t) % map_width)*resolution + origin(0);
        point(1) = floor(centroids0.at(t) / map_width)*resolution + origin(1);

        // for (int w = 0; w < prev_cent_0x.size(); w++)
        // {
        //     // Compare the previous centroids to the current calculated centroid
        //     if ( (fabs( prev_cent_0x.at(w) - point(0)) < 0.01) && (fabs( prev_cent_0y.at(w) - point(1)) < 0.01) )
        //     {
        //         // If the current centroid is too close to a previous centroid, skip
        //         std::cout << "Already went to this centroid " << prev_cent_0x.at(w) << " , " << prev_cent_0y.at(w) << std::endl;
        //         goto bad_centroid;
        //     }
        // }

        if((point(0) < origin(0) + 0.05) && (point(1) < origin(1) + 0.05))
        {
            // If the centroid is too close to the map orgin, skip
            // goto bad_centroid;
            continue;
        }

        else
        {
            // If the centroid is valid, add its x and y values to their respective vectors
            centroid0_Xpts.push_back(point(0));
            centroid0_Ypts.push_back(point(1));
            centroid_pts.push_back({point, points_in_regions.at(t)});
            centroid_grid_pts.push_back(Eigen::Vector2i(centroids0.at(t) % map_width, floor(centroids0.at(t) / map_width)));
            continue;

            // // Determine the distance between the current centroid and the robot's position
            // double delta_x = point(0) - robot0_pose_(0); 
            // double delta_y = point(1) - robot0_pose_(1); 
            // double sum = (pow(delta_x ,2)) + (pow(delta_y ,2));
            // dist0 = pow( sum , 0.5 );

            // // Store the distance value in a vector
            // dist0_arr.push_back(dist0);
        }

        // Skip to the end of the for loop if there was an invalid centroid
        // bad_centroid: 
        // std::cout << "Ignore bad centroid" << std::endl;
    }
}

/*
void FrontExpl::find_closest_centroid()
{
    // Set the first smallest distance to be large
    smallest = 9999999.0;
    for(int u = 0; u < dist0_arr.size(); u++)
    {
        // For each centroid distance, determine if the current centroid is closer than the previous closest centroid

        if (dist0_arr.at(u) < 0.1)
        {
            // If the centroid distance is less than 0.1m, its too close. Ignore it
        }
        else if (dist0_arr.at(u) < smallest)
        {
            // If the current distance element is smaller than the previous closest centroid
            // replace the smallest distance variable with the current distance 
            smallest = dist0_arr.at(u);
            
            // Update the centroid that the robot will move to
            move_to_pt = u;
        }

        else
        {
            // If the current distance value is larger than 0.1 and the smallest_dist value, then ignore it
        }
    }
}
*/

/*
void FrontExpl::edge_index_to_point()
{
    for (int t = 0; t < edge0_vec.size(); t++)
    {
        // For all the frontier edge cells, find the x and y coordinates in the map frame
        point(0) = (edge0_vec.at(t) % map_width)*resolution + origin(0);
        point(1) = floor(edge0_vec.at(t) / map_width)*resolution + origin(1);

        // Add the cells x and y values to their respective vectors
        centroid0_Xpts.push_back(point(0));
        centroid0_Ypts.push_back(point(1));
        centroid_pts.push_back(point);
        centroid_grid_pts.push_back(Eigen::Vector2i(centroids0.at(t) % map_width, floor(centroids0.at(t) / map_width)));

        // Determine the distance between the current frontier edge and the robot's position
        double delta_x = point(0) - robot0_pose_(0); 
        double delta_y = point(1) - robot0_pose_(1); 
        double sum = (pow(delta_x ,2)) + (pow(delta_y ,2));
        dist0 = pow( sum , 0.5 );

        // Store the distance value in a vector
        dist0_arr.push_back(dist0);
    }
}
*/

std::vector<std::pair<Eigen::Vector2d, int>> FrontExpl::run(){
    // std::cout << "Resetting vairbales and clearing all vectors" << std::endl;
    centroid0 = 0;
    centroid0_index = 0;
    dist0_arr.clear();
    edge0_vec.clear();
    neighbor0_index.clear();
    neighbor0_value.clear();
    centroids0.clear();
    centroid0_Xpts.clear();
    centroid0_Ypts.clear();
    centroid_pts.clear();
    centroid_grid_pts.clear();
    frontier_regions.clear();
    points_in_regions.clear();


    // std::cout << "Getting Frontier" << std::endl;


    // If there is no map data and / or start service isnt called, do nothing and instead start the loop over again
    if (FE0_map.size()!=0)
    {
        // Find the frontier edges
        find_all_edges();

        // If there are no frontier edges, skip to the end of the main_loop and try again
        if (edge0_vec.size() == 0)
        {
            // std::cout << "No Frontier" << std::endl;
            goto skip;
        }

        sort( edge0_vec.begin(), edge0_vec.end() );
        edge0_vec.erase( unique( edge0_vec.begin(), edge0_vec.end() ), edge0_vec.end() );

        // Given the forntier edges, find the frontier regions
        find_regions();

        // Given the regions, find the centroids of each region
        find_centroids();

        // Given the centroid vector, convert the controids from map cells to x-y coordinates in the map frame
        // so a goal can be sent to move_base
        centroid_index_to_point();

        // If there are no values in the centroid x or y vectors, find the closest frontier edge to move to
        // if ( ( centroid0_Xpts.size() == 0 ) || ( centroid0_Ypts.size() == 0) )
        // {
        //     centroid0_Xpts.clear();
        //     centroid0_Ypts.clear();
        //     dist0_arr.clear();
        //     std::cout << "Couldnt find a centroid, move to closest edge instead" << std::endl;

        //     // Given the edge vector, convert the edges from map cells to x-y coordinates in the map frame
        //     // so a goal can be sent to move_base                       
        //     edge_index_to_point(); 
        // }

        // std::cout << "" << std::endl;
    }

    // If the size of the map data is 0, then run through the loop again
    else
    {
        // std::cout << "No Map" << std::endl;
    }

    // Skip to the end of the loop if there are no fontier regions
    skip:

    return centroid_pts;
}