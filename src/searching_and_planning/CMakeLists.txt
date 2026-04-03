cmake_minimum_required(VERSION 3.8)
project(searching_and_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(searching_and_planning
  src/core/SearchAndPlan.cpp
  src/core/frontierSearching.cpp
  src/core/MySamplingBasedPlanners.cpp
  src/core/Config.cpp
  src/tools/HelpfulClass.cpp
)

target_include_directories(searching_and_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  searching_and_planning
)
target_link_libraries(searching_and_planning Eigen3::Eigen yaml-cpp)

# Add executable
add_executable(path_planning
  src/single_plan_server.cpp
)

target_link_libraries(path_planning Eigen3::Eigen)
target_link_libraries(path_planning searching_and_planning)

ament_target_dependencies(
  path_planning
  rclcpp
  cartographer_ros_msgs
  geometry_msgs
  nav_msgs
  # Eigen3
)

# Include directories
target_include_directories(path_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  path_planning
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
