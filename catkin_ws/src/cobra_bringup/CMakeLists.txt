cmake_minimum_required(VERSION 3.0.2)
project(cobra_bringup)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories()

#############
## Install ##
#############

# Install launch files
install(FILES
  launch/cobra.launch
  launch/cobra_hardware.launch
  launch/cobra_sim.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml"
)

# Install rviz config files
install(DIRECTORY
  rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)
