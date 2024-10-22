cmake_minimum_required(VERSION 3.5)
project(face_tracker_3000)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  face_tracker_3000/face_tracker_3000_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, if any
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()