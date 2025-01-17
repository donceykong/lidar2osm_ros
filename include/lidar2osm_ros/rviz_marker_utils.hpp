#ifndef RVIZ_MARKER_UTILS_HPP
#define RVIZ_MARKER_UTILS_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Function to create a line marker
visualization_msgs::msg::Marker getLineMarker(const int robot1Num,
                                              const int robot2Num,
                                              const geometry_msgs::msg::TransformStamped &transform1,
                                              const geometry_msgs::msg::TransformStamped &transform2,
                                              const int currentMarkerIndex,
                                              rclcpp::Time current_time);

#endif  // RVIZ_MARKER_UTILS_HPP