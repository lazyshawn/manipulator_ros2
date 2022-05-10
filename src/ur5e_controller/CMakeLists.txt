cmake_minimum_required(VERSION 3.5)
project(ur5e_controller)

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
# find_package (ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies ()
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(shawnlib REQUIRED)
find_package(ur_driver REQUIRED)

set(msg_files
  "msg/JointState.msg"
)
set(srv_files
  "srv/Bringup.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(urdriver
  src/controller.cpp
)
target_include_directories(urdriver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(urdriver rclcpp ur_driver)
rosidl_target_interfaces(urdriver ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(pathPlanner
  src/path_planner.cpp
)
target_include_directories(pathPlanner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pathPlanner rclcpp shawnlib ur_driver)
rosidl_target_interfaces(pathPlanner ${PROJECT_NAME} rosidl_typesupport_cpp)

install(TARGETS
  urdriver
  pathPlanner
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
