cmake_minimum_required(VERSION 3.8)
project(sanad_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sanad_description REQUIRED)
find_package(ros_gz_sim REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  FILES .env .env.example
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

ament_package()
