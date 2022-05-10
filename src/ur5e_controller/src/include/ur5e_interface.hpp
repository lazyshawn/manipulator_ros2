#pragma once

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>
// msg/srv files
#include "ur5e_controller/msg/joint_state.hpp"
#include "ur5e_controller/srv/bringup.hpp"
// normal include files
#include "ur5e_driver.h"

#define SERVOJ_TIME 0.008

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;


class UR5eDriver : public rclcpp::Node {
public:
  UR5eDriver();

private:
  UrDriver urRobot;
  // UR通信的条件变量
  std::condition_variable rt_ur_msg_cond, ur_msg_cond;
  uint8_t robotState = 0;
  std::string targetIP;
  uint16_t portIndex;
  THETA jointStateRec;       // 记录的关节角位置
  double maxDq = 2*deg2rad;  // 一个伺服周期内电机的最大转角

  rclcpp::Service<ur5e_controller::srv::Bringup>::SharedPtr robotBringupSrv_;
  rclcpp::Subscription<ur5e_controller::msg::JointState>::SharedPtr jointStateSub_;
  rclcpp::Publisher<ur5e_controller::msg::JointState>::SharedPtr jointStatePub_;
  rclcpp::TimerBase::SharedPtr jointStatePub_timer;

  void robotBringupSrv_callback(
      const std::shared_ptr<ur5e_controller::srv::Bringup::Request> request,
      std::shared_ptr<ur5e_controller::srv::Bringup::Response> response);
  void jointStateSub_callback(const ur5e_controller::msg::JointState::SharedPtr msg);
  void jointStatePub_timer_callback();
};

/******************************************************************************/
/*******************************************************
* @ Desc: 构造函数
*******************************************************/
UR5eDriver::UR5eDriver() : Node("ur5e_driver") {
  // 机械臂启动服务
  robotBringupSrv_ = create_service<ur5e_controller::srv::Bringup>(
      "ur5e_bringup",
      std::bind(&UR5eDriver::robotBringupSrv_callback, this, _1, _2));
  // 订阅机械臂关节角指令
  jointStateSub_ = create_subscription<ur5e_controller::msg::JointState>(
      "ur5e_servoj", 10,
      std::bind(&UR5eDriver::jointStateSub_callback, this, _1));
  // 发布关节角位置
  jointStatePub_ = create_publisher<ur5e_controller::msg::JointState>(
      "ur5e_joint_state", 10);
  jointStatePub_timer = create_wall_timer(
      8ms, std::bind(&UR5eDriver::jointStatePub_timer_callback, this));
}

/*******************************************************
* @ Desc: 唤醒机械臂服务的回调函数
*******************************************************/
void UR5eDriver::robotBringupSrv_callback(
    const std::shared_ptr<ur5e_controller::srv::Bringup_Request> request,
    std::shared_ptr<ur5e_controller::srv::Bringup_Response> response) {
    if (robotState != 0) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
        "UR5e is already actived, robotState: %d", robotState);
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connecting to UR5e: %s : %ld",
              request->target_ip.c_str(), request->port_index);
  targetIP = request->target_ip;
  portIndex = request->port_index;
  urRobot.bringup(rt_ur_msg_cond, ur_msg_cond, targetIP, portIndex);
  urRobot.start();
  usleep(500); // 机械臂初始化，确保能TCP通信连上(accept)
  urRobot.setServojTime(SERVOJ_TIME);
  urRobot.uploadProg();

  robotState = response->robot_state = 1;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connect to UR5e: %s : %ld",
              request->target_ip.c_str(), request->port_index);
}

/*******************************************************
* @ Desc: 机械臂servoj命令的回调函数
*******************************************************/
void UR5eDriver::jointStateSub_callback(
    const ur5e_controller::msg::JointState::SharedPtr msg) {
  std::vector<double> jointCmd(6,0);
  for (int i = 0; i < 6; ++i) {
    jointCmd[i] = msg->jointstate[i];
    double delQ = abs(jointCmd[i] - jointStateRec[i]);
    if (delQ > maxDq) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
           "Excessive speed at Joint %d: %lf / %lf",
           i, delQ * rad2deg, maxDq * rad2deg);
      return;
    };
  }
  urRobot.servoj(jointCmd, 1);
}

void UR5eDriver::jointStatePub_timer_callback() {
  // 机械臂启动之后开始发布
  if (!robotState) { return; }

  std::vector<double> joint_angle(6);
  auto msg = ur5e_controller::msg::JointState();
  // 获取机械臂关节角位置
  joint_angle = urRobot.rt_interface_->robot_state_->getQActual();
  for (int i=0; i<6; ++i) {
    jointStateRec[i] = msg.jointstate[i] = joint_angle[i];
  }
  jointStatePub_->publish(msg);
}
