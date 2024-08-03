cmake_minimum_required(VERSION 3.8)
project(fyp_ws)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xacro REQUIRED)


include_directories(include)

add_executable(keyboard_control /home/gregory2/fyp_ws/src/examples/keyboard_control.cpp)
ament_target_dependencies(keyboard_control
  rclcpp
  std_msgs
  geometry_msgs
)


install(TARGETS
  keyboard_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  /home/gregory2/fyp_ws/src/launch
  /home/gregory2/fyp_ws/src/config
  /home/gregory2/fyp_ws/src/urdf
  /home/gregory2/fyp_ws/src/worlds
  /home/gregory2/fyp_ws/src/rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

