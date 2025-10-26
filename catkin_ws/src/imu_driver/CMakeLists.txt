cmake_minimum_required(VERSION 3.0.2)
project(imu_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(imu_driver src/imu_driver.cpp)
target_link_libraries(imu_driver ${catkin_LIBRARIES})
