#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "lidar2osm_ros/srv/register_map.hpp"  // Change this to match your package name
#include <Eigen/Dense>

// void matrix_callback(const std::shared_ptr<lidar2osm_ros::srv::RegisterMap::Request> request,
//                       std::shared_ptr<lidar2osm_ros::srv::RegisterMap::Response> response)
// {
//     Eigen::MatrixXd input_matrix(2, 2);
//     input_matrix << request->matrix.data[0], request->matrix.data[1],
//                     request->matrix.data[2], request->matrix.data[3];

//     // Example: Multiply matrix by 2
//     Eigen::MatrixXd output_matrix = input_matrix * 2;

//     response->result.layout.dim.resize(2);
//     response->result.layout.dim[0].size = 2;
//     response->result.layout.dim[1].size = 2;
//     response->result.layout.dim[0].stride = 2;
//     response->result.layout.dim[1].stride = 1;
//     response->result.data = {output_matrix(0,0), output_matrix(0,1),
//                              output_matrix(1,0), output_matrix(1,1)};

//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Matrix processed successfully.");
// }

void matrix_callback(const std::shared_ptr<lidar2osm_ros::srv::RegisterMap::Request> request,
    std::shared_ptr<lidar2osm_ros::srv::RegisterMap::Response> response)
{
    Eigen::MatrixXd input_matrix(2, 2);
    input_matrix << request->matrix.data[0], request->matrix.data[1],
    request->matrix.data[2], request->matrix.data[3];

    // Example: Multiply matrix by 2
    Eigen::MatrixXd output_matrix = input_matrix * 2;

    // Ensure response layout is properly set
    response->result.layout.dim.resize(2);
    response->result.layout.dim[0].size = 2;
    response->result.layout.dim[1].size = 2;
    response->result.layout.dim[0].stride = 2;
    response->result.layout.dim[1].stride = 1;

    // Assign response data
    response->result.data = {
        output_matrix(0, 0), output_matrix(0, 1),
        output_matrix(1, 0), output_matrix(1, 1)
    };

    // Debugging log for response
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Matrix processed successfully.");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response Matrix: [%f, %f, %f, %f]",
    response->result.data[0], response->result.data[1],
    response->result.data[2], response->result.data[3]);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("map_registration_server");

    // auto service = node->create_service<lidar2osm_ros::srv::RegisterMap>("process_matrix", &matrix_callback);

    auto service = node->create_service<lidar2osm_ros::srv::RegisterMap>(
        "process_matrix",
        std::bind(matrix_callback, std::placeholders::_1, std::placeholders::_2)
    );
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to process matrices.");
    rclcpp::spin(node);
    rclcpp::shutdown();
}
