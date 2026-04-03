cmake_minimum_required(VERSION 3.8)
project(path_tracking)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Executables
add_executable(pathTracking src/pathTracking.cpp)
add_executable(pathInterp src/pathInterp.cpp)
add_executable(pathTrackingVLA src/pathTrackingVLA.cpp)

# Link dependencies
ament_target_dependencies(pathTracking rclcpp nav_msgs geometry_msgs Eigen3 tf2 tf2_ros)
ament_target_dependencies(pathInterp rclcpp nav_msgs geometry_msgs Eigen3 tf2 tf2_ros)
ament_target_dependencies(pathTrackingVLA rclcpp nav_msgs geometry_msgs Eigen3 tf2 tf2_ros)

# Install targets
install(TARGETS
  pathTracking
  pathTrackingVLA
  pathInterp
  DESTINATION lib/${PROJECT_NAME}
)

# Install resources
install(DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()