#include <chrono>
#include <cstdlib>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "ur5e_controller/srv/bringup.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("ur5e_bringup_client");
  rclcpp::Client<ur5e_controller::srv::Bringup>::SharedPtr client =
      node->create_client<ur5e_controller::srv::Bringup>("ur5e_bringup");

  auto request = std::make_shared<ur5e_controller::srv::Bringup::Request>();
  request->target_ip = std::string("10.249.181.201");
  request->port_index = uint16_t(50007);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connecting to robot");
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot State Code: %ld",
                result.get()->robot_state);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service ur5e_bringup");
  }

  rclcpp::shutdown();
  return 0;
}

