cmake_minimum_required(VERSION 3.8)
project(scara_4)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
