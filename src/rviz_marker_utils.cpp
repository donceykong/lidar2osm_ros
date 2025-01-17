

#include <cmath>  // For std::sqrt

#include "lidar2osm_ros/rviz_marker_utils.hpp"
#include "lidar2osm_ros/robot.hpp"

visualization_msgs::msg::Marker getLineMarker(const int robot1Num,
                                              const int robot2Num,
                                              const geometry_msgs::msg::TransformStamped& transform1,
                                              const geometry_msgs::msg::TransformStamped& transform2,
                                              const int currentMarkerIndex,
                                              rclcpp::Time current_time) 
{
    // Init robot of interest
    Robot robot1;
    Robot robot2;
    robot1.setRobot(robot1Num + 1);
    robot2.setRobot(robot2Num + 1);

    // Initialize the marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";   // Set the reference frame
    marker.header.stamp = current_time;
    marker.ns = "robot_dist_line";
    marker.id = currentMarkerIndex;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale (line width)
    marker.scale.x = 1.0;

    // Set the color (ave of both robot's colors)
    marker.color.r = (robot1.r + robot2.r) / 2;
    marker.color.g = (robot1.g + robot2.g) / 2;
    marker.color.b = (robot1.b + robot2.b) / 2;
    marker.color.a = 1.0;

    // Define the points (pairs of points form line segments)
    geometry_msgs::msg::Point robot1_position;
    geometry_msgs::msg::Point robot2_position;

    // First line segment
    robot1_position.x = transform1.transform.translation.x; 
    robot1_position.y = transform1.transform.translation.y; 
    robot1_position.z = transform1.transform.translation.z;
    robot2_position.x = transform2.transform.translation.x; 
    robot2_position.y = transform2.transform.translation.y; 
    robot2_position.z = transform2.transform.translation.z;

    marker.points.push_back(robot1_position);
    marker.points.push_back(robot2_position);

    return marker;
}