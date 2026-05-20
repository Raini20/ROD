cmake_minimum_required(VERSION 3.8)
project(rod_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(arm_demo src/arm_demo.cpp)
ament_target_dependencies(arm_demo
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

add_executable(scara_demo src/scara_demo.cpp)
ament_target_dependencies(scara_demo
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

install(TARGETS arm_demo scara_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
