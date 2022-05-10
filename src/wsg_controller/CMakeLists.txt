cmake_minimum_required(VERSION 3.5)
project(wsg_controller)

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

set(msg_files
  "msg/Position.msg"
)
set(srv_files
  "srv/Bringup.srv"
)
# 根据 msg/srv 文件生成对应的头文件
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)
# 编译运行时的 interface 依赖 (使用当前包的消息时需要)
ament_export_dependencies(rosidl_default_runtime)

add_executable(wsgDriver
  src/wsg_driver.cpp
  src/wsg_main.cpp
)
# 添加头文件位置
target_include_directories(wsgDriver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# 为节点链接需要的包
ament_target_dependencies(wsgDriver rclcpp std_msgs)
# 使用 interface
rosidl_target_interfaces(wsgDriver ${PROJECT_NAME} rosidl_typesupport_cpp)

install(TARGETS
  wsgDriver
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
