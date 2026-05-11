cmake_minimum_required(VERSION 3.8)
project(rod_scene)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
