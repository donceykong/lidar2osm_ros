#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MapRecieverNode : public rclcpp::Node {
public:
  // Constructor
  MapRecieverNode(const std::string& robot_name) 
    : Node(robot_name + "_map_reciever_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
      // Create list of all other robot names
      std::array<std::string, 4> allRobotNames = {"robot1", "robot2", "robot3", "robot4"};
      std::array<std::string, 3> robotTeamNames;
      std::copy_if(allRobotNames.begin(), allRobotNames.end(), robotTeamNames.begin(),
                             [&robot_name](const std::string& name) { return name != robot_name; });

      // node params
      this->declare_parameter<std::string>("ego_frame", robot_name + "_base_link");
      this->declare_parameter<std::string>("teammate_b_frame", robotTeamNames[0] + "_base_link");
      this->declare_parameter<double>("min_dist_threshold", 2.0);
      this->declare_parameter<double>("eff_comms_dist_threshold", 10.0);
      
      this->get_parameter("ego_frame", ego_frame_);
      this->get_parameter("teammate_b_frame", teammate_b_frame_);
      this->get_parameter("min_dist_threshold", min_dist_threshold_);
      this->get_parameter("eff_comms_dist_threshold", eff_comms_dist_threshold_);

      // Timer to run at 10 Hz
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MapRecieverNode::check_distance, this));
  }

private:
void check_distance() {
  try {
      // Get transforms of both frames relative to a common frame (e.g., "map")
      // RCLCPP_INFO(this->get_logger(), "Frames '%s' and '%s'", ego_frame_.c_str(), teammate_b_frame_.c_str());

      geometry_msgs::msg::TransformStamped tf_a = tf_buffer_.lookupTransform("world", ego_frame_, tf2::TimePointZero);
      geometry_msgs::msg::TransformStamped tf_b = tf_buffer_.lookupTransform("world", teammate_b_frame_, tf2::TimePointZero);

      // Compute Euclidean distance
      double dx = tf_a.transform.translation.x - tf_b.transform.translation.x;
      double dy = tf_a.transform.translation.y - tf_b.transform.translation.y;
      double dz = tf_a.transform.translation.z - tf_b.transform.translation.z;
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

      // Check if within threshold
      if (distance <= eff_comms_dist_threshold_ && distance >= min_dist_threshold_) {
          RCLCPP_INFO(this->get_logger(), "Frames '%s' and '%s' are within threshold: %f meters", 
          ego_frame_.c_str(), teammate_b_frame_.c_str(), distance);
      }
  } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
}

// ROS 2 timer
rclcpp::TimerBase::SharedPtr timer_;

// TF2 listener and buffer
tf2_ros::Buffer tf_buffer_;
tf2_ros::TransformListener tf_listener_;

// Parameters
std::string ego_frame_;
std::string teammate_b_frame_;
double min_dist_threshold_;
double eff_comms_dist_threshold_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
  
    if (argc < 2) {
      RCLCPP_ERROR(rclcpp::get_logger("map_reciever_node"), "Usage: map_reciever_node <robot_name>");
      return 1;
    }
  
    std::string robot_name = argv[1];
    rclcpp::spin(std::make_shared<MapRecieverNode>(robot_name));
    rclcpp::shutdown();
    return 0;
  }