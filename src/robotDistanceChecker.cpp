#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

class SphereDrawerNode : public rclcpp::Node
{
public:
    SphereDrawerNode()
        : Node("sphere_drawer_node"), tf_buffer_(), tf_listener_(tf_buffer_)
    {
        // Declare parameters
        this->declare_parameter<std::string>("frame1", "frame1");
        this->declare_parameter<std::string>("frame2", "frame2");
        this->declare_parameter<double>("distance_threshold", 1.0);

        // Get parameters
        this->get_parameter("frame1", frame1_);
        this->get_parameter("frame2", frame2_);
        this->get_parameter("distance_threshold", distance_threshold_);

        // Publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("sphere_marker", 10);

        // Timer to periodically check the distance and publish markers
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SphereDrawerNode::timerCallback, this));
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
                publishSphere(transform1, transform2);
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    void publishSphere(const geometry_msgs::msg::TransformStamped &transform1,
                       const geometry_msgs::msg::TransformStamped &transform2)
    {
        // Compute the center of the sphere
        double center_x = (transform1.transform.translation.x + transform2.transform.translation.x) / 2.0;
        double center_y = (transform1.transform.translation.y + transform2.transform.translation.y) / 2.0;
        double center_z = (transform1.transform.translation.z + transform2.transform.translation.z) / 2.0;

        // Create a sphere marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "sphere";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = center_x;
        marker.pose.position.y = center_y;
        marker.pose.position.z = center_z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = distance_threshold_ * 2.0; // Diameter of the sphere
        marker.scale.y = distance_threshold_ * 2.0;
        marker.scale.z = distance_threshold_ * 2.0;
        marker.color.a = 0.5; // Transparency
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // Publish the marker
        marker_pub_->publish(marker);
    }

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
    rclcpp::spin(std::make_shared<SphereDrawerNode>());
    rclcpp::shutdown();
    return 0;
}
