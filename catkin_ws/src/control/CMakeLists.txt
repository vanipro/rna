cmake_minimum_required(VERSION 3.0.2)
project(control)

add_compile_options(-std=c++14 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  controller_manager
  joint_state_controller
  diff_drive_controller
  robot_state_publisher
  serial
)

find_package(nlohmann_json 3.2.0 QUIET)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    controller_manager
    joint_state_controller
    diff_drive_controller
    robot_state_publisher
    serial
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})

if(nlohmann_json_FOUND)
  target_link_libraries(controller nlohmann_json::nlohmann_json)
endif()

install(TARGETS controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml")

catkin_install_python(PROGRAMS	
  scripts/udp_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

