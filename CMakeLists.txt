cmake_minimum_required(VERSION 3.8)
project(lidar2osm_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(include)

# Robot distance checker node
add_executable(robot_distance_checker 
  src/robot_distance_checker.cpp 
  src/rviz_marker_utils.cpp
)

# Link the executable to the required dependencies
ament_target_dependencies(robot_distance_checker rclcpp sensor_msgs nav_msgs tf2_ros visualization_msgs)

# Install the node executable
install(TARGETS
  robot_distance_checker
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

# Install the header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

# Install the launch files
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
