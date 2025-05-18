cmake_minimum_required(VERSION 3.8)
project(interfacing)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install Python scripts
install(
  PROGRAMS
    scripts/Gazebo_h1.py
    scripts/Gazebo_Inside_Follow.py
    scripts/Gazebo_Outside_Follow.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install model folders
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

