cmake_minimum_required(VERSION 3.8)
project(sanad_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Room.msg"
  "msg/Device.msg"
  "srv/GetRoom.srv" 
  "srv/GetAllRooms.srv"
  "srv/DeleteRoom.srv"
  "srv/AddRoom.srv"
  "srv/UpdateRoom.srv"
  "srv/NavigateToRoom.srv"
  "srv/RegisterDevice.srv"
  "srv/DeleteDevice.srv"
 )
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
