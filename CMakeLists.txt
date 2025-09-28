cmake_minimum_required(VERSION 3.5)
project(arl_ros2_project)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  arl_ros2_project/turtle_commander.py
  arl_ros2_project/shape_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
