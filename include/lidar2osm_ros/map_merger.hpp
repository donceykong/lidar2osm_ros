/*
 * This node will take in a two pointclouds and two poses. One set for the target and one set for the source.alignas
 * It will return the relative pose from the target to the source cloud.
*/
#ifndef MAP_MERGER_HPP
#define MAP_MERGER_HPP

class MapMergerNode : public rclcpp::Node {
  public:
    // Constructor
    MapRecieverNode(const std::string& robot_name) 
      : Node(robot_name + "_map_reciever_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {};
      
      void checkPeerDistance(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {};
  private:
    void publishPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud) {};
    // ROS 2 timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    std::string robot_name_;
    std::string ego_frame_;
    std::string teammate_b_name_;
    std::string teammate_b_frame_;
    double min_dist_threshold_;
    double eff_comms_dist_threshold_;
};

#endif  // MAP_MERGER_HPP