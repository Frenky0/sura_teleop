cmake_minimum_required(VERSION 3.8)
project(sura_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sura_msgs REQUIRED)

add_executable(cirtesub_teleop
  src/cirtesub_teleop.cpp
)

add_executable(blueboat_teleop
  src/blueboat_teleop.cpp
)

ament_target_dependencies(cirtesub_teleop
  ament_index_cpp
  controller_manager_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

ament_target_dependencies(blueboat_teleop
  controller_manager_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  sura_msgs
)

install(
  TARGETS cirtesub_teleop blueboat_teleop
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
