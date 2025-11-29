cmake_minimum_required(VERSION 3.8)
project(team_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找裁判系统要求的依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(referee_pkg REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 编译STAGE_1节点
add_executable(vision_node_stage1 src/vision_node.cpp)
ament_target_dependencies(vision_node_stage1
  rclcpp
  sensor_msgs
  referee_pkg
  cv_bridge
  OpenCV
  geometry_msgs
  std_msgs
)

# 安装节点（裁判系统可识别路径）
install(TARGETS
  vision_node_stage1
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件（可选，用于参数调优）
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

