#include <chrono>
#include <cstdlib>
#include <memory>
#include "rclcpp/rclcpp.hpp"
// msg/srv files
#include "ur5e_controller/msg/joint_state.hpp"
#include "ur5e_controller/srv/bringup.hpp"
// normal include files
#include "include/ur5e_driver.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

#define SERVOJ_TIME 0.008
UrDriver urRobot;
// UR通信的条件变量
std::condition_variable rt_ur_msg_cond, ur_msg_cond;
uint8_t robotState = 0;
std::string targetIP;
uint16_t portIndex;

/*******************************************************
* @ Service:  启动夹爪
* @ Request:  target_ip, port_index
* @ Response: grip_state
*******************************************************/
void bringup(
    const std::shared_ptr<ur5e_controller::srv::Bringup_Request> request,
    std::shared_ptr<ur5e_controller::srv::Bringup_Response> response) {
  if (robotState != 0) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "UR5e is already actived, robotState: %d", robotState);
  }
  targetIP = request->target_ip;
  portIndex = request->port_index;
  // c++中定义了一个类的对象之后，还能再调用对象的构造函数吗? - 夏之幻想的回答
  // https://www.zhihu.com/question/365133268/answer/968623573
  new (&urRobot) UrDriver(rt_ur_msg_cond, ur_msg_cond, targetIP, portIndex);
  urRobot.start();
  usleep(500); // 机械臂初始化，确保能TCP通信连上(accept)
  urRobot.setServojTime(SERVOJ_TIME);
  urRobot.uploadProg();

  robotState = response->robot_state = 1;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connect UR5e: %s : %ld",
              request->target_ip.c_str(), request->port_index);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]",
              (long int)response->robot_state);
}

/*******************************************************
* @ Topic: 运动控制
* @ Msgs:  jointstate
*******************************************************/
void topic_callback(const ur5e_controller::msg::JointState::SharedPtr msg) {
  std::vector<double> jointState(6,0);
  for (int i=0; i<6; ++i) {
    jointState[i] = msg->jointstate[i];
    std::cout << jointState[i] << std::endl;
  }
  urRobot.servoj(jointState, 1);
}

/*******************************************************
* @ Topic: 发布机械臂关节角度信息
* @ Msgs:  jointstate
*******************************************************/

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ur5e_driver");

  rclcpp::Service<ur5e_controller::srv::Bringup>::SharedPtr service =
      node->create_service<ur5e_controller::srv::Bringup>("ur5e_bringup",
                                                          &bringup);
  rclcpp::Subscription<ur5e_controller::msg::JointState>::SharedPtr Subscription =
      node->create_subscription<ur5e_controller::msg::JointState>(
          "ur5e_joint_cmd", 10, std::bind(&topic_callback, _1));
  rclcpp::Publisher<ur5e_controller::msg::JointState>::SharedPtr Publisher =
      node->create_publisher<ur5e_controller::msg::JointState>("ur5e_joint_state", 10);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node <ur5e_driver> is ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

