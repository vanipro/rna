#!/bin/bash

BAG_NAME="cobra_$(date +%Y%m%d_%H%M%S)"
mkdir -p ~/rosbags
rosbag record -O ~/rosbags/$BAG_NAME.bag /scan /tf /tf_static /joint_states /cobra_base_controller/cmd_vel /cobra_base_controller/odom /cobra_base_controller/state /clock
