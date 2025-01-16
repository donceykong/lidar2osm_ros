#ifndef RVIZ_MARKER_UTILS_HPP
#define RVIZ_MARKER_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Function to create a sphere marker
visualization_msgs::msg::Marker getSphereMarker(rclcpp::Time current_time,
                                                const double &normalized_distance, 
                                                const double &distance,
                                                const geometry_msgs::msg::Point &center_point);

// Function to create a line marker
visualization_msgs::msg::Marker getLineMarker(const int robotNum,
                                              const int currentMarkerIndex,
                                              rclcpp::Time current_time,
                                              const double &normalized_distance, 
                                              const geometry_msgs::msg::Point &center_point, 
                                              const geometry_msgs::msg::TransformStamped &transform);

#endif  // RVIZ_MARKER_UTILS_HPP
