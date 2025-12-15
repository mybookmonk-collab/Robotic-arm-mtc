cmake_minimum_required(VERSION 3.8)
project(panda_moveit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# 添加可执行文件
add_executable(panda_moveit_control src/panda_moveit_control.cpp)
ament_target_dependencies(panda_moveit_control 
  ament_cmake
  Boost
  Eigen3
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  rclcpp moveit_ros_planning_interface)
target_link_libraries(panda_moveit_control
  moveit_ros_planning_interface::moveit_move_group_interface
)
# 安装可执行文件
install(TARGETS
  panda_moveit_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
