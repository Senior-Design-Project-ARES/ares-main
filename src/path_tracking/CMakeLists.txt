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
add_executable(telemBinary src/telemBinaryTool.cpp)
add_library(telemBinaryShared src/telemBinary.cpp)
add_library(telemUartShared src/telemUart.cpp)

target_include_directories(telemBinaryShared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(telemUartShared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(pathTrackingVLA telemBinaryShared)
target_link_libraries(pathTrackingVLA telemUartShared)
target_link_libraries(telemBinary telemBinaryShared)

# Link dependencies
ament_target_dependencies(pathTracking rclcpp nav_msgs geometry_msgs Eigen3 tf2 tf2_ros)
ament_target_dependencies(pathInterp rclcpp nav_msgs geometry_msgs Eigen3 tf2 tf2_ros)
ament_target_dependencies(pathTrackingVLA rclcpp nav_msgs geometry_msgs Eigen3 tf2 tf2_ros)
ament_target_dependencies(telemBinary rclcpp nav_msgs geometry_msgs Eigen3 tf2 tf2_ros)

set_target_properties(
  pathTracking
  pathInterp
  pathTrackingVLA
  telemBinary
  telemBinaryShared
  telemUartShared
  PROPERTIES CXX_STANDARD 17
)

# Install targets
install(TARGETS
  pathTracking
  pathTrackingVLA
  pathInterp
  telemBinary
  DESTINATION lib/${PROJECT_NAME}
)

# Install resources
install(DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()