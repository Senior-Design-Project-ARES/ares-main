#include "CSpace.h"

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    // std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    // MyGridCSpace2D& cspace = *cspace_ptr;

    std::unique_ptr<amp::GridCSpace2D> cspace_ptr = std::make_unique<amp::GridCSpace2D>(m_cells_per_dim, m_cells_per_dim, -M_PI, M_PI, -M_PI, M_PI);
    amp::GridCSpace2D& cspace = *cspace_ptr;
    double step = 2*M_PI/m_cells_per_dim;
    double half_step = step/2;
    double critical = 0.01;
    bool crash = false;

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    for(int i = 0; i < m_cells_per_dim; i++){
        Eigen::Vector2d state = Eigen::Vector2d(i*step+half_step -M_PI, 0);
        Eigen::Vector2d joint1 = manipulator.getJointLocation(state, 1);
        crash = check_collision(env,joint1);
        if(!crash){
            crash = check_collision_between_point(env, manipulator.getBaseLocation(), joint1, critical);
        }

        for(int j = 0; j < m_cells_per_dim; j++){
            if (crash){
                cspace(i,j) = true;
                continue;
            }

            state[1] = j*step+half_step-M_PI;
            Eigen::Vector2d end_effector = manipulator.getJointLocation(state, 2);
            if (check_collision(env, end_effector)){
                cspace(i,j) = true;
                continue;
            }
            cspace(i,j) = check_collision_between_point(env, joint1, end_effector, critical);
        }
        crash = false;
    }

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

bool MyManipulatorCSConstructor::check_collision(const amp::Environment2D& env, Eigen::Vector2d point) {
    amp::Obstacle2D obstacle;
    double cross_product;
    bool possible_collision = 1;
    for (int ob_idx = 0; ob_idx < env.obstacles.size(); ++ob_idx) {
        obstacle = env.obstacles[ob_idx];
        

        int n = obstacle.verticesCCW().size();
        for (int vertix_idx = 0; vertix_idx < n; ++vertix_idx) {
            const Eigen::Vector2d& v1 = obstacle.verticesCCW()[vertix_idx];
            const Eigen::Vector2d& v2 = obstacle.verticesCCW()[(vertix_idx + 1) % n];
            Eigen::Vector2d edge = v1 - v2;
            Eigen::Vector2d point_to_v1 = point - v1;
            cross_product = edge(0) * point_to_v1(1) - edge(1) * point_to_v1(0);
            if (cross_product > 0){
                possible_collision = 0;
                break; // Point is outside of this obstacle
            }
            // std::cout << "Cross product with edge " << vertix_idx << ": " << cross_product << std::endl;
        }
        if (!possible_collision) {
            possible_collision = 1;
            continue; // Check the next obstacle
        }
        // std::cout << "Collision with obstacle " << ob_idx << std::endl;
        // std::cout << "Collision: True" << std::endl;
        return true; // Point is inside this obstacle
    }
    return false; // Point is outside all polygons
}

bool MyManipulatorCSConstructor::check_collision_between_point(const amp::Environment2D& env, Eigen::Vector2d point1, Eigen::Vector2d point2, double critical){
    Eigen::Vector2d first2second = point2 - point1;
    double previous_step = first2second.norm();
    double devide = 2;

    while(previous_step > critical){
        for(double k = 1; k < devide; k+=2){
            if (check_collision(env, point1 + first2second*(k/devide))){
                return true;
            }
        }
        devide = devide*2;
        previous_step = previous_step/2;
    }
    return false;
}

std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<amp::GridCSpace2D> cspace_ptr = std::make_unique<amp::GridCSpace2D>(cells_x_dim, cells_y_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    amp::GridCSpace2D& cspace = *cspace_ptr;
    double cell_width = (env.x_max-env.x_min)/cells_x_dim;
    double cell_height = (env.y_max-env.y_min)/cells_y_dim;
    check_collision_enviroument check_collision(env);
    std::cout << "Constructing C-space for point agent" << std::endl;

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    for(int cell_n = 0; cell_n < cells_x_dim; cell_n++){
        for(int cell_m = 0; cell_m < cells_y_dim; cell_m++){
            for(int i = 0; i < 10; i++){
                double random_x = env.x_min+cell_width*cell_n+(rand()%98+1)/100.0*cell_width;
                double random_y = env.y_min+cell_height*cell_m+(rand()%98+1)/100.0*cell_height;
                if (check_collision.all(Eigen::Vector2d(random_x, random_y))){
                    cspace(cell_n, cell_m) = true;
                    break;
                }
            }
        }
    }
    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

std::vector<int8_t> MyPointAgentCSConstructor::construct1D(const amp::Environment2D& env){
    std::unique_ptr<amp::GridCSpace2D> grid_cspace_ptr = construct(env);
    const amp::GridCSpace2D& grid_cspace = *grid_cspace_ptr;
    std::vector<int8_t> cspace_1D;

    for(int i = 0; i < cells_x_dim; i++){
        for(int j = 0; j < cells_y_dim; j++){
            cspace_1D.push_back(int8_t(grid_cspace(j,i)));
        }
    }
    return cspace_1D;
}

const amp::GridCSpace2D_T<int8_t>& MyLidarEmulateConstructor::construct4point(Eigen::Vector2d location){
    removeFrontierFromMap();
    double cell_width = (env.x_max-env.x_min)/cells_x();
    double cell_height = (env.y_max-env.y_min)/cells_y();
    check_collision_enviroument check_collision(env);

    for(int cell_n = 0; cell_n < cells_x(); cell_n++){
        for(int cell_m = 0; cell_m < cells_y(); cell_m++){
            double x_center = env.x_min+cell_width*cell_n + cell_width/2;
            double y_center = env.y_min+cell_height*cell_m + cell_height/2;

            if (map(cell_n, cell_m) == -1){
                if (pow(x_center - location(0), 2) + pow(y_center - location(1), 2) < LIDARRADIUS){
                    for(int i = 0; i < 10; i++){
                        double random_x = x_center - cell_width/2 +(rand()%98+1)/100.0*cell_width;
                        double random_y = y_center - cell_height/2 +(rand()%98+1)/100.0*cell_height;
                        if (check_collision.all(Eigen::Vector2d(random_x, random_y))){
                            map(cell_n, cell_m) = 1;
                            break;
                        }
                        map(cell_n, cell_m) = 0;
                    }
                }
            }
        }
    }
    return map;
}

const amp::GridCSpace2D_T<int8_t>& MyLidarEmulateConstructor::lidarMimicConstruct4point(Eigen::Vector2d location){
    removeFrontierFromMap();
    double cell_width = (env.x_max-env.x_min)/cells_x();
    double cell_height = (env.y_max-env.y_min)/cells_y();
    check_collision_enviroument check_collision(env);
    std::vector<std::pair<Eigen::Vector2d, double>> obstacle_location;

    Eigen::Vector2d arm = Eigen::Vector2d(0,1);
    for(double angle = 0; angle < 2*M_PI; angle += ANGLERESOLUSION){
        Eigen::Vector2d add_on =  (Eigen::Rotation2Dd(angle) * arm).normalized();

        for(double distance = 0; distance < LIDARRADIUS; distance+=cell_height/2){
            Eigen::Vector2d location_check = location+add_on*distance;

            if(location_check(0) <= env.x_min || location_check(0) >= env.x_max ||
               location_check(1) <= env.y_min || location_check(1) >= env.y_max){
                continue;
            }

            if(getState(location_check) != -1){
                continue;
            }
            if(checkCellCollision(location_check, check_collision)){
                obstacle_location.push_back({location_check, distance});
                // updateAroundPoint(location_check, 1, distance);
                break;
            }
            updateAroundPoint(location_check, 0, distance);
        }
    }
    for(int i = 0; i < obstacle_location.size(); i++){
        updateAroundPoint(obstacle_location[i].first, 1, obstacle_location[i].second);
    }
    return map;
}

const amp::GridCSpace2D_T<int8_t>& MyLidarEmulateConstructor::lidarMimicConstruct4point2(Eigen::Vector2d location){
    removeFrontierFromMap();
    double cell_width = (env.x_max-env.x_min)/cells_x();
    double cell_height = (env.y_max-env.y_min)/cells_y();
    check_collision_enviroument check_collision(env);
    std::vector<std::pair<Eigen::Vector2d, double>> obstacle_location;
    Eigen::Vector2d arm = Eigen::Vector2d(0,1);

    for(double angle = 0.0000001; angle < 2*M_PI*3; angle += ANGLERESOLUSION){
        Eigen::Vector2d add_on =  (Eigen::Rotation2Dd(angle) * arm).normalized();
        auto [i, j] = map.getCellFromPoint(location(0), location(1));
        double nextBoundaryX = (add_on(0) > 0) 
            ? env.x_min + (i + 1) * cell_width 
            : env.x_min + i * cell_width;
        double nextBoundaryY = (add_on(1) > 0) 
            ? env.y_min + (j + 1) * cell_height 
            : env.y_min + j * cell_height;

        double tMaxX = (add_on(0) != 0)
            ? (nextBoundaryX - location(0)) / add_on(0)
            : std::numeric_limits<double>::infinity();
        double tMaxY = (add_on(1) != 0)
            ? (nextBoundaryY - location(1)) / add_on(1)
            : std::numeric_limits<double>::infinity();

        double tDeltaX = (add_on(0) != 0)
            ? cell_width / std::abs(add_on(0))
            : std::numeric_limits<double>::infinity();
        double tDeltaY = (add_on(1) != 0)
            ? cell_height / std::abs(add_on(1))
            : std::numeric_limits<double>::infinity();

        double traveled = 0.0;

        while (traveled < LIDARRADIUS) {
            Eigen::Vector2d p = location + add_on * traveled;

            if (check_collision.all(p)) {
                obstacle_location.push_back({p, traveled});
                break;
            } else {
                updateAroundPoint(p, 0, traveled); // free
            }

            // Move to next boundary
            if (tMaxX < tMaxY) {
                i += (add_on(0) > 0) ? 1 : -1;
                traveled = tMaxX;
                tMaxX += tDeltaX;
            } else {
                j += (add_on(1) > 0) ? 1 : -1;
                traveled = tMaxY;
                tMaxY += tDeltaY;
            }
        }
    }
    for(int i = 0; i < obstacle_location.size(); i++){
        updateAroundPoint(obstacle_location[i].first, 1, obstacle_location[i].second);
    }
    return map;
}

bool MyLidarEmulateConstructor::checkCellCollision(Eigen::Vector2d location_check, check_collision_enviroument& check_collision){
    auto[x,y] = map.getCellFromPoint(location_check(0), location_check(1));
    double step_x = (map.x0Bounds().second - map.x0Bounds().first)/cells_x();
    double step_y = (map.x1Bounds().second - map.x1Bounds().first)/cells_y();

    for(int i = 0; i < SAMPLE_POINT; i++){
        for (int j = 0; j < SAMPLE_POINT; j++){
            double random_x = map.x0Bounds().first + x*step_x + double(rand())/RAND_MAX*step_x;
            double random_y = map.x1Bounds().first + y*step_y + double(rand())/RAND_MAX*step_y;
            if (check_collision.all(Eigen::Vector2d(random_x, random_y))){
                return true;
            }
        }
    }
    return false;
}

void MyLidarEmulateConstructor::updateAroundPoint(const Eigen::Vector2d& location_check, int8_t value, double radius){
    if(location_check(0) <= env.x_min || location_check(0) >= env.x_max ||
       location_check(1) <= env.y_min || location_check(1) >= env.y_max){
        return;
    }

    std::pair<int, int> temp = map.getCellFromPoint(location_check(0),location_check(1));
    int cell_x = temp.first;
    int cell_y = temp.second;

    if(radius == 0){
        map(cell_x, cell_y) = value;
        return;
    }

    int num_point_to_change = std::ceil(radius*ANGLERESOLUSION);
    if (num_point_to_change != 1){
        DEBUG("num_point_to_change: " << num_point_to_change);
    }

    for (int i = 0; i < num_point_to_change/2 + 1; i++ ){
        for(int j = 0; j < num_point_to_change/2 + 1; j++){
            if (cell_x + i < cells_x() && cell_y + j < cells_y()) {
                map(cell_x + i, cell_y + j) = value;
            }

            if (cell_x + i < cells_x() && cell_y - j >= 0) {
                map(cell_x + i, cell_y - j) = value;
            }

            if (cell_x - i >= 0 && cell_y + j < cells_y()) {
                map(cell_x - i, cell_y + j) = value;
            }

            if (cell_x - i >= 0 && cell_y - j >= 0) {
                map(cell_x - i, cell_y - j) = value;
            }
        }
    }
}

const std::vector<int8_t>& MyLidarEmulateConstructor::construct4point1D(Eigen::Vector2d location){
    // construct4point(location);
    // lidarMimicConstruct4point(location);
    lidarMimicConstruct4point2(location);
    map_1D.clear();

    for(int i = 0; i < cells_x(); i++){
        for(int j = 0; j < cells_y(); j++){
            map_1D.push_back(int8_t(map(j,i)));
        }
    }
    return map_1D;
}

const amp::GridCSpace2D_T<int8_t>& MyLidarEmulateConstructor::getMapptr(){
    return map;
}

void MyLidarEmulateConstructor::addFrontierToMap(const std::vector<Eigen::Vector2i>& frontier_pts){
    for (const auto& pt : frontier_pts) {
        if (pt(0) >= 0 && pt(0) < cells_x() && pt(1) >= 0 && pt(1) < cells_y()) {
            map(pt(0), pt(1)) = 2; // Mark as frontier
        }
    }
    last_frontier = frontier_pts;
}

void MyLidarEmulateConstructor::removeFrontierFromMap(){
    for (const auto& pt : last_frontier) {
        if (pt(0) >= 0 && pt(0) < cells_x() && pt(1) >= 0 && pt(1) < cells_y()) {
            map(pt(0), pt(1)) = 0; // Reset to free space
        }
    }
    last_frontier.clear();
}

void MyLidarEmulateConstructor::reset(){
    for(int i = 0; i < cells_x(); i++){
        for(int j = 0; j < cells_y(); j ++){
            map(i, j) = -1;
        }
    }
}

int MyLidarEmulateConstructor::getState(const Eigen::Vector2d& location){
    const amp::GridCSpace2D_T<int8_t>& map = getMapptr();
    auto[i, j] = map.getCellFromPoint(location(0), location(1));
    return map(i, j);
}
