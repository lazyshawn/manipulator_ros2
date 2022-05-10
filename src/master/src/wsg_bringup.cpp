#include <chrono>
#include <cstdlib>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "wsg_controller/srv/bringup.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // if (argc != 2) {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
  //   return 1;
  // }

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("wsg_bringup_client");
  rclcpp::Client<wsg_controller::srv::Bringup>::SharedPtr client =
      node->create_client<wsg_controller::srv::Bringup>("wsg_bringup");

  auto request = std::make_shared<wsg_controller::srv::Bringup::Request>();
  request->target_ip = std::string("10.249.180.222");
  request->port_index = uint16_t(1000);

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
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->grip_state);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service wsg_bringup");
  }

  rclcpp::shutdown();
  return 0;
}

