#include <array>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "lidar2osm_ros/rviz_marker_utils.hpp"
#include "lidar2osm_ros/robot.hpp"

class robotDistanceCheckerNode : public rclcpp::Node
{
public:
    robotDistanceCheckerNode()
        : Node("robot_distance_checker_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        // Declare & get param
        this->declare_parameter<double>("distance_threshold", 10.0);
        this->get_parameter("distance_threshold", distance_threshold_);

        // Publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("sphere_marker", 100);
        robot_boundary_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_boundary_markers", 100);
        robot_collision_sphere_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("robot_collision_markers", 100);

        // Timer to periodically check the distance and publish markers
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
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

    // TODO: Move to visualization cpp
    void drawCollisionSphere(int robot1Num, int robot2Num) {
        // Compute the center of the sphere
        geometry_msgs::msg::Point center_point;
        center_point.x = (currentRobotTFs[robot1Num].transform.translation.x + currentRobotTFs[robot2Num].transform.translation.x) / 2.0;
        center_point.y = (currentRobotTFs[robot1Num].transform.translation.y + currentRobotTFs[robot2Num].transform.translation.y) / 2.0;
        center_point.z = (currentRobotTFs[robot1Num].transform.translation.z + currentRobotTFs[robot2Num].transform.translation.z) / 2.0;

        int x = int(center_point.x);
        int y = int(center_point.y);
        int z = int(center_point.z);

        // Create a sphere marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = current_time;
        marker.ns = std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z); //"robot" + std::to_string(robot1Num + 1) +"_robot" + std::to_string(robot2Num + 1) + "_collision_spheres";
        marker.id = 0; //collisionSphereIndex;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = distance_threshold_; // Diameter of the sphere
        marker.scale.y = distance_threshold_;
        marker.scale.z = distance_threshold_;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;
        collisionSphereIndex++;
        
        // Pub
        robot_collision_sphere_pub_->publish(marker);
    }

    void checkDistance(int robot1Num, int robot2Num) {
        double distance = getDistance(currentRobotTFs[robot1Num], currentRobotTFs[robot2Num]);

        // Check if the distance is within the threshold
        if  (distance < 0.5) 
        {
            drawCollisionSphere(robot1Num, robot2Num);
        }
        else if (distance <= distance_threshold_)
        {
            addMarkerToArray(robot1Num, robot2Num);
        }
    }

    void checkAllRobotBounds() {
        for (int robot1Num = 0; robot1Num < numRobots; robot1Num++) {
            for (int robot2Num = robot1Num + 1; robot2Num < numRobots; robot2Num++) {
                checkDistance(robot1Num, robot2Num);
            }
        }
    }

    // TODO: Move to visualization cpp
    void publishRobotBoundary(int robotNum) {
        // Init robot of interest
        Robot robot;
        robot.setRobot(robotNum + 1);

        // Create a sphere marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = current_time;
        marker.ns = "robotBoundary" + std::to_string(robotNum + 1);
        marker.id = robotNum;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = currentRobotTFs[robotNum].transform.translation.x;
        marker.pose.position.y = currentRobotTFs[robotNum].transform.translation.y;
        marker.pose.position.z = currentRobotTFs[robotNum].transform.translation.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = distance_threshold_; // Diameter of the sphere
        marker.scale.y = distance_threshold_;
        marker.scale.z = distance_threshold_;
        marker.color.r = robot.r;
        marker.color.g = robot.g;
        marker.color.b = robot.b;
        marker.color.a = 0.3;
        
        // Pub
        robot_boundary_marker_pub_->publish(marker);
    }

    void publishRobotBoundaries() {
        for (int robotNum = 0; robotNum < numRobots; robotNum++) {
            publishRobotBoundary(robotNum);
        }
    }

    void updateTransforms() {
        for (int robotNum = 0; robotNum < numRobots; robotNum++) {
            std::string frame_name = "robot" + std::to_string(robotNum + 1) + "_base_link";
            currentRobotTFs[robotNum] = tf_buffer_.lookupTransform("world", frame_name, tf2::TimePointZero);
        }
    }

    void timerCallback()
    {
        try
        {
            // Set curr time
            current_time = rclcpp::Clock().now();
            updateTransforms();
            publishRobotBoundaries();
            checkAllRobotBounds();
        }
        catch (const tf2::TransformException &ex)
        {
            // RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    void addMarkerToArray(int robot1Num, int robot2Num)
    {
        if (currentCommsMarkerIndex == (maxNumMarkers - 1)) {
            currentCommsMarkerIndex = 0;
        }

        visualization_msgs::msg::Marker lineMarker = getLineMarker(robot1Num, robot2Num, currentRobotTFs[robot1Num], currentRobotTFs[robot2Num], currentCommsMarkerIndex, current_time);
        marker_pub_->publish(lineMarker);

        currentCommsMarkerIndex++;
    }

    // Current Time
    rclcpp::Time current_time = rclcpp::Clock().now();
    static constexpr int maxNumMarkers = 1000;
    int currentCommsMarkerIndex = 0;
    int collisionSphereIndex = 0;

    // Number of robots
    static const int numRobots = 4;
    std::array<geometry_msgs::msg::TransformStamped, numRobots> currentRobotTFs;

    // Parameters
    double distance_threshold_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ROS publishers and timers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_boundary_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_collision_sphere_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robotDistanceCheckerNode>());
    rclcpp::shutdown();
    return 0;
}
