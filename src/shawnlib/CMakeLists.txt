cmake_minimum_required(VERSION 3.5)
project(shawnlib)

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

# find dependencies
find_package(ament_cmake REQUIRED)

add_library(shawn_lib SHARED
  src/user_interface.cpp
)
target_include_directories(shawn_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# 为了使下游文件可以访问
ament_export_targets(shawn_lib HAS_LIBRARY_TARGET)
# 注册 导出头文件
install(
  DIRECTORY include/  # 需要共享的头文件位置
  DESTINATION include
)
# 注册 导出库文件
install(
  TARGETS shawn_lib   # 告诉ros2有这么个目标（可执行文件或者库）
  EXPORT shawn_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
# createlib 包后续的 target 链接 shawn_lib
# target_link_libraries(bin shawn_lib)


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
