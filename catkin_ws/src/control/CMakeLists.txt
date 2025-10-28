cmake_minimum_required(VERSION 3.0.2)
project(control)

add_compile_options(-std=c++14 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2_geometry_msgs
    tf2_ros
    tf2
    controller_manager
    robot_state_publisher
    diff_drive_controller
    joint_state_controller
    serial
)

find_package(nlohmann_json 3.2.0 QUIET)

add_service_files(
  FILES
  DriverCmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2_geometry_msgs
    tf2_ros
    tf2
    controller_manager
    robot_state_publisher
    diff_drive_controller
    joint_state_controller
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
