#include "include/ur5e_interface.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UR5eDriver>());
  rclcpp::shutdown();
  return 0;
}

