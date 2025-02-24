#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

class LidarMapNode : public rclcpp::Node {
public:
  LidarMapNode(const std::string& robot_name) 
    : Node(robot_name + "_global_map_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
      // Parameters
      this->declare_parameter<std::string>("map_frame", "world");
      this->declare_parameter<double>("resolution", 5.0);

      // Construct dynamic topics using robot_name
      pointcloud_topic_ = "/" + robot_name + "/ouster/semantic_points";
      robot_frame_ = robot_name + "_base_link";

      // Retrieve parameters
      map_frame_ = this->get_parameter("map_frame").as_string();
      resolution_ = this->get_parameter("resolution").as_double();

      // Subscriber and publishers
      pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          pointcloud_topic_, rclcpp::SensorDataQoS(),
          std::bind(&LidarMapNode::pointCloudCallback, this, std::placeholders::_1));
      pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/" + robot_name + "/global_pointcloud", 10);

      // Timer for publishing the map and pointcloud
      timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&LidarMapNode::publishMapAndPointCloud, this));
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    try {
      // Get the transform from the robot frame to the map frame
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_.lookupTransform(map_frame_, cloud_msg->header.frame_id, tf2::TimePointZero);

      // Convert PointCloud2 to PCL
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromROSMsg(*cloud_msg, *cloud);

      // Transform the cloud to the map frame
      Eigen::Affine3d transform = tf2::transformToEigen(transform_stamped.transform);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::transformPointCloud(*cloud, *cloud_transformed, transform);

      // Voxel Downsample
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(cloud_transformed);
      voxel_filter.setLeafSize(resolution_, resolution_, resolution_); // Leaf size in meters
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>());
      voxel_filter.filter(*cloud_downsampled);

      // Aggregate the point cloud
      *aggregated_cloud_ += *cloud_downsampled;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
  }

  void publishMapAndPointCloud() {
    // Publish the aggregated point cloud
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*aggregated_cloud_, cloud_msg);
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.header.frame_id = map_frame_;
    pointcloud_pub_->publish(cloud_msg);
  }

  // Parameters
  std::string pointcloud_topic_;
  std::string map_frame_;
  std::string robot_frame_;
  double resolution_;

  // ROS 2 components
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Transform listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Global point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aggregated_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // if (argc < 2) {
  //   RCLCPP_ERROR(rclcpp::get_logger("global_map_node"), "Usage: lidar_map_node <robot_name>");
  //   return 1;
  // }

  std::string robot_name = argv[1];
  rclcpp::spin(std::make_shared<LidarMapNode>(robot_name));
  rclcpp::shutdown();
  return 0;
}

