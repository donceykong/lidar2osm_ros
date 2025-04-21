#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "lidar2osm_ros/srv/register_map.hpp"
#include <Eigen/Dense>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("map_registration_client");
    auto client = node->create_client<lidar2osm_ros::srv::RegisterMap>("process_matrix");

    auto request = std::make_shared<lidar2osm_ros::srv::RegisterMap::Request>();

    // Create a sample 2x2 Eigen matrix
    Eigen::MatrixXd input_matrix(2, 2);
    input_matrix << 1.0, 2.0, 3.0, 4.0;

    request->matrix.layout.dim.resize(2);
    request->matrix.layout.dim[0].size = 2;
    request->matrix.layout.dim[1].size = 2;
    request->matrix.layout.dim[0].stride = 2;
    request->matrix.layout.dim[1].stride = 1;
    request->matrix.data = {input_matrix(0,0), input_matrix(0,1),
                            input_matrix(1,0), input_matrix(1,1)};

    while (!client->wait_for_service(3s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // RCLCPP_INFO(node->get_logger(), "Received response: [%f, %f, %f, %f]",
        //         result.get()->result[0], result.get()->result[1],
        //         result.get()->result[2], result.get()->result[3]);
        // RCLCPP_INFO(node->get_logger(), "WORKED!.");
        // Assuming the response contains a vector 'result' with float elements
        // Access the response
        auto response = result.get();

        // The data is stored in response->result.data, which is a vector
        const auto& result_data = response->result.data;

        // Check if the result data is empty
        if (!result_data.empty()) {
            for (size_t i = 0; i < result_data.size(); ++i) {
                RCLCPP_INFO(node->get_logger(), "Received response[%zu]: %f", i, result_data[i]);
            }
        } else {
            RCLCPP_WARN(node->get_logger(), "Received an empty result.");
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to receive response.");
    }
  
    rclcpp::shutdown();
    return 0;
}