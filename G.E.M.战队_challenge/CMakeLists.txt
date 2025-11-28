cmake_minimum_required(VERSION 3.5)
project(gem_challenge)

# 编译选项
add_compile_options(-Wall -Wextra -Wpedantic -O3)

# 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(referee_pkg REQUIRED)    # 裁判系统消息（必须）
find_package(vision_msgs QUIET)       # 万一你要用，不强制
find_package(std_msgs REQUIRED)

# include 目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# -------------------------
#   vision_node (Level 1/2)
# -------------------------
add_executable(vision_node
  src/vision_node.cpp
)

ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  referee_pkg
)

# -------------------------
#   shooter_node (Level 3)
#   没写 shooter_node.cpp 的话这段不会报错
# -------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/shooter_node.cpp")
  add_executable(shooter_node
    src/shooter_node.cpp
  )

  ament_target_dependencies(shooter_node
    rclcpp
    sensor_msgs
    geometry_msgs
    cv_bridge
    OpenCV
    referee_pkg
  )
endif()

# -------------------------
# 安装可执行文件
# -------------------------
install(
  TARGETS
    vision_node
    shooter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# -------------------------
# 安装 launch / config
# -------------------------
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------
ament_package()