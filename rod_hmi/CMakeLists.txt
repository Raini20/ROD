cmake_minimum_required(VERSION 3.8)
project(rod_hmi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
set(IMGUI_SOURCES
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/imgui_impl_opengl3.cpp
)

add_executable(rod_hmi src/rod_hmi.cpp ${IMGUI_SOURCES})

target_include_directories(rod_hmi PRIVATE ${IMGUI_DIR})

target_link_libraries(rod_hmi OpenGL::GL glfw)

ament_target_dependencies(rod_hmi
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  ament_index_cpp
)

install(TARGETS rod_hmi DESTINATION lib/${PROJECT_NAME})

ament_package()
