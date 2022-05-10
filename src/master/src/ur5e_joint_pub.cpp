#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ur5e_controller/msg/joint_state.hpp"
#include "master/user_interface.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("ur5e_joint_pub") {
    publisher_ = this->create_publisher<ur5e_controller::msg::JointState>(
        "ur5e_servoj", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = ur5e_controller::msg::JointState();
    std::vector<double> ori_angle = {0, -98.9, 117.8, -108.9, -90, 90};
    std::vector<double> off_angle = {0, -98.9, 117.8, -108.9, -90, 70};
    char command = scanKeyboard();
    switch(command) {
      case 'h':
        for (int i=0; i<6; ++i) {
          msg.jointstate[i] = ori_angle[i]*M_PI/180;
        }
        publisher_->publish(msg);
        break;
      case 'g':
        for (int i=0; i<6; ++i) {
          msg.jointstate[i] = off_angle[i]*M_PI/180;
        }
        publisher_->publish(msg);
        break;
      case 27:
        return;
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ur5e_controller::msg::JointState>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
