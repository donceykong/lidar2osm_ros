#include <array>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

class robotDistanceCheckerNode : public rclcpp::Node
{
public:
    robotDistanceCheckerNode()
        : Node("robot_distance_checker_node"),
        tf_buffer_(this->get_clock()), // Pass the node's clock
        tf_listener_(tf_buffer_)
    {
        // Declare parameters
        this->declare_parameter<std::string>("frame1", "robot1_base_link");
        this->declare_parameter<std::string>("frame2", "robot2_base_link");
        this->declare_parameter<double>("distance_threshold", 10.0);

        // Get parameters
        this->get_parameter("frame1", frame1_);
        this->get_parameter("frame2", frame2_);
        this->get_parameter("distance_threshold", distance_threshold_);

        // Publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("sphere_marker", 10);

        // Timer to periodically check the distance and publish markers
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&robotDistanceCheckerNode::timerCallback, this)
        );
    }


private:
    void timerCallback()
    {
        try
        {
            // Get the transforms for the two frames
            geometry_msgs::msg::TransformStamped transform1 = tf_buffer_.lookupTransform("world", frame1_, tf2::TimePointZero);
            geometry_msgs::msg::TransformStamped transform2 = tf_buffer_.lookupTransform("world", frame2_, tf2::TimePointZero);

            // Compute the distance between the two frames
            double dx = transform1.transform.translation.x - transform2.transform.translation.x;
            double dy = transform1.transform.translation.y - transform2.transform.translation.y;
            double dz = transform1.transform.translation.z - transform2.transform.translation.z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            // Check if the distance is within the threshold
            if (distance <= distance_threshold_)
            {
                if (currentMarkerIndex == (maxNumMarkers-1)) {
                    RCLCPP_INFO(this->get_logger(), "The marker array count is: %d. Setting index to 0.", maxNumMarkers);
                    currentMarkerIndex = 0;
                }

                addMarkerToArray(transform1, transform2, distance);
                publishMarkers();
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    void publishMarkers() {
        for (int i=0; i<=currentMarkerIndex; i++) {
            marker_pub_->publish(robotDistMarkers[i]);
        }
    }

    void addMarkerToArray(const geometry_msgs::msg::TransformStamped &transform1,
                          const geometry_msgs::msg::TransformStamped &transform2,
                          const double distance)
    {
        // Compute the center of the sphere
        geometry_msgs::msg::Point center_point;
        center_point.x = (transform1.transform.translation.x + transform2.transform.translation.x) / 2.0;
        center_point.y = (transform1.transform.translation.y + transform2.transform.translation.y) / 2.0;
        center_point.z = (transform1.transform.translation.z + transform2.transform.translation.z) / 2.0;

        // Closer the robots are, then the closer to 1.0 this value will be.
        double normalized_distance = (distance_threshold_ - distance) / distance_threshold_;

        // // Create a sphere marker
        // visualization_msgs::msg::Marker marker;
        // marker.header.frame_id = "world";
        // marker.header.stamp = this->now();
        // marker.ns = "sphere";
        // marker.id = currentMarkerIndex;
        // marker.type = visualization_msgs::msg::Marker::SPHERE;
        // marker.action = visualization_msgs::msg::Marker::ADD;
        // marker.pose.position.x = center_x;
        // marker.pose.position.y = center_y;
        // marker.pose.position.z = center_z;
        // marker.pose.orientation.x = 0.0;
        // marker.pose.orientation.y = 0.0;
        // marker.pose.orientation.z = 0.0;
        // marker.pose.orientation.w = 1.0;
        // marker.scale.x = distance_threshold_ * 2.0; // Diameter of the sphere
        // marker.scale.y = distance_threshold_ * 2.0;
        // marker.scale.z = distance_threshold_ * 2.0;
        // marker.color.a = 0.5;
        // marker.color.r = normalized_distance;
        // marker.color.g = normalized_distance;
        // marker.color.b = normalized_distance;

        visualization_msgs::msg::Marker lineMarker = getLineMarker(normalized_distance, center_point, transform1, transform2);
        robotDistMarkers[currentMarkerIndex] = lineMarker;
        currentMarkerIndex++;
    }

    visualization_msgs::msg::Marker getLineMarker(const double &normalized_distance, 
                                                  const geometry_msgs::msg::Point &center_point, 
                                                  const geometry_msgs::msg::TransformStamped &transform1, 
                                                  const geometry_msgs::msg::TransformStamped &transform2) 
    {
        // Initialize the marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";   // Set the reference frame
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "robot_distance_line";
        marker.id = currentMarkerIndex;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale (line width)
        marker.scale.x = 0.05;

        // Set the color (red with full alpha)
        marker.color.r = 1.0 - normalized_distance;
        marker.color.g = normalized_distance;
        marker.color.b = 0.0;
        marker.color.a = 1.0 - normalized_distance;

        // Define the points (pairs of points form line segments)
        geometry_msgs::msg::Point robot1_position, robot2_position;

        // First line segment
        robot1_position.x = transform1.transform.translation.x; 
        robot1_position.y = transform1.transform.translation.y; 
        robot1_position.z = transform1.transform.translation.z;
        marker.points.push_back(center_point);
        marker.points.push_back(robot1_position);

        // Second line segment
        robot2_position.x = transform2.transform.translation.x; 
        robot2_position.y = transform2.transform.translation.y; 
        robot2_position.z = transform2.transform.translation.z;
        marker.points.push_back(center_point);
        marker.points.push_back(robot2_position);

        return marker;
    }

    // Marker Array
    static constexpr int maxNumMarkers = 10000;
    std::array<visualization_msgs::msg::Marker, maxNumMarkers> robotDistMarkers;
    int currentMarkerIndex = 0;

    // Parameters
    std::string frame1_;
    std::string frame2_;
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
