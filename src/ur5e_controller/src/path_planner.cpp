#include <chrono>
#include <cstdlib>
#include <memory>
#include "rclcpp/rclcpp.hpp"

// package dependencies
#include "shawnlib/user_interface.hpp"
// msg/srv files
#include "ur5e_controller/msg/joint_state.hpp"
// normal include files
#include <queue>
#include "include/ur5e_driver.h"
#include "include/path_planner.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

THETA jointState;
std::queue<THETA> jointTraj;

class PathPlanner : public rclcpp::Node {
  public:
    PathPlanner();

  private:
    rclcpp::Subscription<ur5e_controller::msg::JointState>::SharedPtr jointStateSub_;
    rclcpp::Subscription<ur5e_controller::msg::JointState>::SharedPtr trajCmdSub_;
    rclcpp::Publisher<ur5e_controller::msg::JointState>::SharedPtr trajPub_;
    rclcpp::TimerBase::SharedPtr trajPub_timer;
    rclcpp::TimerBase::SharedPtr test_timer;
    rclcpp::CallbackGroup::SharedPtr reentrant_group; 

    void trajPub_timer_callback();
    void test_timer_callback();
    void jointStateSub_callback(const ur5e_controller::msg::JointState::SharedPtr msg);
};

PathPlanner::PathPlanner() : Node("path_planner"){
  reentrant_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  jointStateSub_ = create_subscription<ur5e_controller::msg::JointState>(
      "ur5e_joint_state", 10, std::bind(&PathPlanner::jointStateSub_callback, this, _1));
  trajPub_ = create_publisher<ur5e_controller::msg::JointState>(
      "ur5e_servoj", 10);
  trajPub_timer = create_wall_timer(
      8ms, std::bind(&PathPlanner::trajPub_timer_callback, this), reentrant_group);
  test_timer = create_wall_timer(
      1s, std::bind(&PathPlanner::test_timer_callback, this), reentrant_group);
}

void PathPlanner::trajPub_timer_callback() {
  if (jointTraj.empty()) { return; }
  auto msg = ur5e_controller::msg::JointState();
  // 给消息赋值
  THETA jntCmd = jointTraj.front();
  jointTraj.pop();
  for (int i=0; i<6; ++i) {
    msg.jointstate[i] = jntCmd[i];
  }
  trajPub_->publish(msg);
}

// 订阅机械臂关节角状态
void PathPlanner::jointStateSub_callback(
    const ur5e_controller::msg::JointState::SharedPtr msg) {
  for (int i=0; i<6; ++i) {
    jointState[i] = msg->jointstate[i];
  }
}

bool traj_interpolate(THETA origVal, THETA goalVal, double prop, THETA &refVal){
  if (prop >= 1) { return true; }

  uint8_t interpMode = INTERP_3JI;
  // 按插补模式在关节空间进行轨迹插补
  switch (interpMode) {
  // sin 関数による軌道補間 | sin函数的轨迹插补
  case INTERP_SIN:
    for (int i = 0; i < 6; ++i) {
      refVal[i] = origVal[i] + (goalVal[i] - origVal[i]) * prop -
          (goalVal[i] - origVal[i]) * sin(2.0 * M_PI * prop) / (2.0 * M_PI);
    }
    break;
  // 1 次関数による軌道補間 | 一次函数的轨迹插补
  case INTERP_1JI:
    for (int i = 0; i < 6; ++i) {
      refVal[i] = origVal[i] + (goalVal[i] - origVal[i]) * prop;
    }
    break;
  // 3 次関数による軌道補間 | 3次函数的轨迹插补
  case INTERP_3JI:
    for (int i = 0; i < 6; ++i) {
      refVal[i] = origVal[i] + (goalVal[i] - origVal[i]) * prop * prop * (3.0 - 2 * prop);
    }
    break;
  // 5 次関数による軌道補間 | 5次函数的轨迹插补
  case INTERP_5JI:
    for (int i = 0; i < 6; ++i) {
      refVal[i] = origVal[i] + (goalVal[i] - origVal[i]) * prop * prop * prop * 
        (10.0 + prop * (-15.0 + 6.0 * prop));
    }
    break;
  // ステップ関数による軌道補間 | 阶跃函数的轨迹插补
  case INTERP_STEP:
    for (int i = 0; i < 6; ++i) {
      refVal[i] = goalVal[i];
    }
    break;
  default:
    printf("\n==>> Error! Unknow interpolate mode!\n");
    break;
  } // switch(interpMode)
  return false;
}

void PathPlanner::test_timer_callback() {
  THETA home_angle = {0, -98.9*deg2rad, 117.8*deg2rad, -108.9*deg2rad, -90*deg2rad, 90*deg2rad};
  THETA go_angle = {0, -98.9*deg2rad, 117.8*deg2rad, -108.9*deg2rad, -90*deg2rad, 70*deg2rad};
  THETA jntCmd, origJoint;
  double prop;
  bool flag;

  uint8_t command = scanKeyboard();
  switch(command) {
    case 'h':
      origJoint = jointState;
      flag = false;
      prop = SERVO_TIME/5;
      while (!flag) {
        flag = ::traj_interpolate(origJoint, home_angle, prop, jntCmd);
        jointTraj.emplace(jntCmd);
        prop += SERVO_TIME/5;
      }
      break;
    case 'g':
      origJoint = jointState;
      flag = false;
      prop = SERVO_TIME/5;
      while (!flag) {
        flag = ::traj_interpolate(origJoint, go_angle, prop, jntCmd);
        jointTraj.emplace(jntCmd);
        prop += SERVO_TIME/5;
      }
      break;
    default:
      break;
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<PathPlanner>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

