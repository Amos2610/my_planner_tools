cmake_minimum_required(VERSION 3.8)
project(my_planner_tools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xarm_msgs REQUIRED)

add_executable(xarm6_planner_joint src/xarm6_planner_joint.cpp)
ament_target_dependencies(xarm6_planner_joint rclcpp xarm_msgs)

install(TARGETS
  xarm6_planner_joint
  DESTINATION lib/${PROJECT_NAME})

ament_package()
