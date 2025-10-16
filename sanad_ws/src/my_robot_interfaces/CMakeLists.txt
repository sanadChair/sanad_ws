# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.8)

# Define the project name
project(my_robot_interfaces)

# Find necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interface files from .msg and .srv definitions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Turtle.msg"
  "msg/TurtleArray.msg"
  "srv/CatchTurtle.srv"
)

# This command is necessary to make the package discoverable by ROS 2
ament_package()