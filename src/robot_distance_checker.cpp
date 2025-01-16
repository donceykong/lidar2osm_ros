#include <array>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "lidar2osm_ros/rviz_marker_utils.hpp"

class robotDistanceCheckerNode : public rclcpp::Node
{
public:
    robotDistanceCheckerNode()
        : Node("robot_distance_checker_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        // Declare parameters
        this->declare_parameter<std::string>("frame1", "robot1_base_link");
        this->declare_parameter<std::string>("frame2", "robot2_base_link");
        this->declare_parameter<std::string>("frame3", "robot3_base_link");
        this->declare_parameter<std::string>("frame4", "robot4_base_link");
        this->declare_parameter<double>("distance_threshold", 10.0);

        // Get parameters
        this->get_parameter("frame1", frame1_);
        this->get_parameter("frame2", frame2_);
        this->get_parameter("frame3", frame3_);
        this->get_parameter("frame4", frame4_);
        this->get_parameter("distance_threshold", distance_threshold_);

        // Publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("sphere_marker", 10);

        // Timer to periodically check the distance and publish markers
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&robotDistanceCheckerNode::timerCallback, this)
        );
    }


private:
    double getDistance(geometry_msgs::msg::TransformStamped transform1, geometry_msgs::msg::TransformStamped transform2) 
    {
        // Compute the distance between the two frames
        double dx = transform1.transform.translation.x - transform2.transform.translation.x;
        double dy = transform1.transform.translation.y - transform2.transform.translation.y;
        double dz = transform1.transform.translation.z - transform2.transform.translation.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);  

        return distance;
    }

    void timerCallback()
    {
        try
        {
            // Get the transforms for the two frames
            rclcpp::Time current_time = rclcpp::Clock().now();
            geometry_msgs::msg::TransformStamped transform1 = tf_buffer_.lookupTransform("world", frame1_, tf2::TimePointZero);
            geometry_msgs::msg::TransformStamped transform2 = tf_buffer_.lookupTransform("world", frame2_, tf2::TimePointZero);
            geometry_msgs::msg::TransformStamped transform3 = tf_buffer_.lookupTransform("world", frame3_, tf2::TimePointZero);

            double distance = getDistance(transform1, transform2);

            // Check if the distance is within the threshold
            if (distance <= distance_threshold_)
            {
                if (currentMarkerIndex == (maxNumMarkers-1)) {
                    RCLCPP_INFO(this->get_logger(), "The marker array count is: %d. Setting index to 0.", maxNumMarkers);
                    currentMarkerIndex = 0;
                }

                addMarkerToArray(transform1, transform2, distance, current_time);
                publishMarkers();
            }
        }
        catch (const tf2::TransformException &ex)
        {
            // RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    void publishMarkers() {
        for (int i=0; i<=currentMarkerIndex; i++) {
            marker_pub_->publish(robotDistMarkers[i]);
        }
    }

    void addMarkerToArray(const geometry_msgs::msg::TransformStamped &transform1,
                          const geometry_msgs::msg::TransformStamped &transform2,
                          const double &distance,
                          rclcpp::Time current_time)
    {
        // Compute the center of the sphere
        geometry_msgs::msg::Point center_point;
        center_point.x = (transform1.transform.translation.x + transform2.transform.translation.x) / 2.0;
        center_point.y = (transform1.transform.translation.y + transform2.transform.translation.y) / 2.0;
        center_point.z = (transform1.transform.translation.z + transform2.transform.translation.z) / 2.0;

        // Closer the robots are, then the closer to 1.0 this value will be.
        double normalized_distance = (distance_threshold_ - distance) / distance_threshold_;

        // visualization_msgs::msg::Marker sphereMarker = getSphereMarker(normalized_distance, distance, center_point);
        // robotDistMarkers[currentMarkerIndex] = sphereMarker;
        // currentMarkerIndex++;

        visualization_msgs::msg::Marker lineMarker = getLineMarker(1, currentMarkerIndex, current_time, normalized_distance, center_point, transform1);
        robotDistMarkers[currentMarkerIndex] = lineMarker;
        currentMarkerIndex++;

        visualization_msgs::msg::Marker lineMarker2 = getLineMarker(2, currentMarkerIndex, current_time, normalized_distance, center_point, transform2);
        robotDistMarkers[currentMarkerIndex] = lineMarker2;
        currentMarkerIndex++;
    }

    // Marker Array
    static constexpr int maxNumMarkers = 10000;
    std::array<visualization_msgs::msg::Marker, maxNumMarkers> robotDistMarkers;
    int currentMarkerIndex = 0;

    // Parameters
    std::string frame1_, frame2_, frame3_, frame4_;
    double distance_threshold_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ROS publishers and timers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robotDistanceCheckerNode>());
    rclcpp::shutdown();
    return 0;
}
