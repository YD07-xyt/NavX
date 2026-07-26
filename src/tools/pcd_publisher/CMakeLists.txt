cmake_minimum_required(VERSION 3.8)
project(pcd_publisher)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(pcd_publisher)

install(PROGRAMS
  pcd_publisher/_launch_wrapper.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pcd_publisher_node
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
