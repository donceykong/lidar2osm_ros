cmake_minimum_required(VERSION 3.8)
project(lidar2osm_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)

# Headers
include_directories(include)

# ==========================  Executables ==========================

# Robot distance checker node
add_executable(robot_distance_checker src/robot_distance_checker.cpp src/rviz_marker_utils.cpp)
ament_target_dependencies(robot_distance_checker rclcpp sensor_msgs nav_msgs tf2_ros visualization_msgs)

# Robot map accumulator node
add_executable(robot_map_accumulator src/robot_map_accumulator.cpp)
ament_target_dependencies(robot_map_accumulator rclcpp sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs tf2_eigen pcl_conversions)

# Robot map reciever node
add_executable(robot_map_reciever src/robot_map_reciever.cpp)
ament_target_dependencies(robot_map_reciever rclcpp sensor_msgs nav_msgs tf2_ros tf2_eigen)

# target_link_libraries(robot_map_accumulator
#   tf2
#   tf2_ros
#   tf2_eigen
# )

# ==========================  Install shared files ==========================

# Install the headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

# Install URDF in share dir
install(DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
)

# Install rviz dir in share
install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
)

# Install the launch files
install(DIRECTORY launch 
    DESTINATION share/${PROJECT_NAME}/launch
)

# Install the node executable
install(TARGETS
  robot_distance_checker
  robot_map_accumulator
  robot_map_reciever
  DESTINATION lib/${PROJECT_NAME}
)

# Install config dir
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# ========================== Test / build ==========================

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
