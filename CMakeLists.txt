cmake_minimum_required(VERSION 3.14)
project(example_cpp)

set(example_cpp_MAJOR_VERSION 0)
set(example_cpp_MINOR_VERSION 0)
set(example_cpp_PATCH_VERSION 0)
set(example_cpp_VERSION
  ${example_cpp_MAJOR_VERSION}.${example_cpp_MINOR_VERSION}.${example_cpp_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(rclcpp REQUIRED)
add_executable(node_01 src/node_01.cpp)
ament_target_dependencies(node_01 rclcpp)
install(TARGETS
  node_01
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
