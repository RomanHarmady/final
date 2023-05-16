cmake_minimum_required(VERSION 3.8)
project(final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rttest REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

include_directories(
  include
)

add_executable(rt_node src/rt_node.cpp)
add_executable(rate_rt_node src/rate_rt_node.cpp)
add_executable(logger src/logger.cpp)

ament_target_dependencies(rt_node
  rclcpp
  std_msgs
  rttest
  realtime_tools
)

ament_target_dependencies(rate_rt_node
  rclcpp
  std_msgs
  rttest
  realtime_tools
)

ament_target_dependencies(logger
  rclcpp
  std_msgs
  rttest
  realtime_tools
)

install(TARGETS
  rate_rt_node
  rt_node
  logger
  DESTINATION lib/${PROJECT_NAME})

ament_package()
