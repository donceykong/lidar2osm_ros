

#include <cmath>  // For std::sqrt

#include "lidar2osm_ros/rviz_marker_utils.hpp"
#include "lidar2osm_ros/robot.hpp"

visualization_msgs::msg::Marker getSphereMarker(rclcpp::Time current_time,
                                                const double &normalized_distance,
                                                const int currentMarkerIndex,
                                                const double &distance,
                                                const geometry_msgs::msg::Point &center_point)
{
    // Create a sphere marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = current_time;
    marker.ns = "sphere";
    marker.id = currentMarkerIndex;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = center_point.x;
    marker.pose.position.y = center_point.y;
    marker.pose.position.z = center_point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = distance; // Diameter of the sphere
    marker.scale.y = distance;
    marker.scale.z = distance;
    marker.color.a = normalized_distance;
    marker.color.r = normalized_distance;
    marker.color.g = normalized_distance;
    marker.color.b = normalized_distance;

    return marker;
}

visualization_msgs::msg::Marker getLineMarker(const int robotNum,
                                              const int currentMarkerIndex,
                                              rclcpp::Time current_time,
                                              const double &normalized_distance, 
                                              const geometry_msgs::msg::Point &center_point, 
                                              const geometry_msgs::msg::TransformStamped &transform) 
{
    // Init robot of interest
    Robot robot;
    robot.setRobot(robotNum);

    // Initialize the marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";   // Set the reference frame
    marker.header.stamp = current_time;
    marker.ns = "robot_distance_line" + currentMarkerIndex;
    marker.id = currentMarkerIndex;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale (line width)
    marker.scale.x = 0.5*normalized_distance;

    // Set the color (red with full alpha)
    marker.color.r = robot.r;
    marker.color.g = robot.g;
    marker.color.b = robot.b;
    marker.color.a = 1.0;

    // Define the points (pairs of points form line segments)
    geometry_msgs::msg::Point robot_position;

    // First line segment
    robot_position.x = transform.transform.translation.x; 
    robot_position.y = transform.transform.translation.y; 
    robot_position.z = transform.transform.translation.z;
    marker.points.push_back(center_point);
    marker.points.push_back(robot_position);

    return marker;
}