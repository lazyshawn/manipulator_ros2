#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "wsg_controller/msg/position.hpp"
#include "master/user_interface.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("joint_cmd_publisher") {
    publisher_ = this->create_publisher<wsg_controller::msg::Position>(
        "wsg_motion_cmd", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = wsg_controller::msg::Position();
    char command = scanKeyboard();
    switch(command) {
      case 'y':
        msg.pos = 50;
        break;
      case 'o':
        msg.pos = 100;
        break;
      case 27:
        return;
    }
    msg.vel = 20;
    publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<wsg_controller::msg::Position>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
