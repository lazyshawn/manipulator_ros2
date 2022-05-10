cmake_minimum_required(VERSION 3.5)
project(master)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############################################################################
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ur5e_controller REQUIRED)
find_package(wsg_controller REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
)
ament_export_dependencies(rosidl_default_runtime)


add_executable(wsg_pub src/wsg_pub.cpp)
target_include_directories(wsg_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(wsg_pub rclcpp wsg_controller)
rosidl_target_interfaces(wsg_pub
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(wsg_bringup src/wsg_bringup.cpp)
ament_target_dependencies(wsg_bringup rclcpp std_msgs wsg_controller)
rosidl_target_interfaces(wsg_bringup
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(ur5e_bringup src/ur5e_bringup.cpp)
ament_target_dependencies(ur5e_bringup rclcpp std_msgs ur5e_controller)
rosidl_target_interfaces(ur5e_bringup
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(ur5e_pub
  src/ur5e_joint_pub.cpp
)
ament_target_dependencies(ur5e_pub rclcpp ur5e_controller)
rosidl_target_interfaces(ur5e_pub ${PROJECT_NAME} rosidl_typesupport_cpp)
target_include_directories(ur5e_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  wsg_pub
  wsg_bringup
  ur5e_bringup
  ur5e_pub
  DESTINATION lib/${PROJECT_NAME})
###############################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
