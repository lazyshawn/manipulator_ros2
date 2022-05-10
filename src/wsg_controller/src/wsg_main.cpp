#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "wsg_controller/wsg_driver.h"
#include "wsg_controller/srv/bringup.hpp"
#include "wsg_controller/msg/position.hpp"

#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

WSGGripper wsg;
std::string targetIP;
uint16_t portIndex;
float vel = 10;
uint8_t gripperState = 0;

/*******************************************************
* @ Service:  启动夹爪
* @ Request:  target_ip, port_index
* @ Response: grip_state
*******************************************************/
void bringup(
    const std::shared_ptr<wsg_controller::srv::Bringup_Request> request,
    std::shared_ptr<wsg_controller::srv::Bringup_Response> response) {
  if (gripperState != 0) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "WSGGripper is already actived, gripperState: %d", gripperState);
  }
  targetIP = request->target_ip;
  portIndex = request->port_index;
  wsg.active(targetIP, portIndex);

  gripperState = response->grip_state = 1;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connect WSGGripper: %s : %ld",
              request->target_ip.c_str(), request->port_index);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]",
              (long int)response->grip_state);
}

/*******************************************************
* @ Topic: 运动控制
* @ Msgs:  pos, vel
*******************************************************/
void topic_callback(const wsg_controller::msg::Position::SharedPtr msg) {
  float pos = msg->pos;
  float localVel = msg->vel < 0 ? vel : msg->vel;
  wsg.move(pos, localVel);
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("wsg_controller");

  rclcpp::Service<wsg_controller::srv::Bringup>::SharedPtr service =
      node->create_service<wsg_controller::srv::Bringup>(
          "wsg_bringup", &bringup);
  rclcpp::Subscription<wsg_controller::msg::Position>::SharedPtr Subscription =
      node->create_subscription<wsg_controller::msg::Position>(
          "wsg_motion_cmd", 10, std::bind(&topic_callback, _1));

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node <wsg_controller> is ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}

