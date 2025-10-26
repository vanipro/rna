cms
exit
export ROS_MASTER_URI=http://192.168.11.153:11311
cms
rostopic lis
rostopic list
hostname -I
export ROS_IP=192.168.65.3
rostopic pub /cobra/mobile_base_controller/cmd_vel geometry_msgs/Twist -r 10 "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
rostopic list
rostopic pub /cobra/mobile_base_controller/cmd_vel geometry_msgs/Twist -r 10 "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
export ROS_HOSTNAME=192.168.65.3
rostopic pub /cobra/mobile_base_controller/cmd_vel geometry_msgs/Twist -r 10 "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
rostopic pub /hello std_msgs/String "data: 'hello'" 
echo $ROS_IP
echo $ROS_HOSTNAME
hostname -I
export ROS_IP=192.168.11.153
export ROS_HOSTNAME=192.168.11.153
rostopic pub /hello std_msgs/String "data: 'hello'" 
export ROS_HOSTNAME=192.168.11.199
rostopic pub /hello std_msgs/String "data: 'hello'" 
echo $ROS_IP
export ROS_IP=192.168.11.199
export ROS_HOSTNAME=192.168.11.199
rostopic pub /hello std_msgs/String "data: 'hello'" 
echo $ROS_IP
echo $ROS_HOSTNAME 
echo $ROS_MASTER_URI 
rostopic list
exit
cmd=s
cms
export ROS_IP=192.168.11.199
export ROS_HOSTNAME=192.168.11.199
rostopic pub /hello std_msgs/String "data: 'hello'" 
export ROS_MASTER_URI=http://192.168.11.153:11311
rostopic pub /hello std_msgs/String "data: 'hello'" 
rostopic list
cms
rosservice call /driver "T: 600
rosservice call /driver "T: 0
body:
- '"T":132,"IO1":255,"IO2":255'" 
rosservice call /driver "T: 132
body:
- '"IO1":255,"IO2":255'" 
rosservice call /driver "T: 132
body:
- 'IO1:255,IO2:255'" 
rosservice call /driver "T: 132
body:
- 'IO1:255,IO2:255'" 
rosservice call /driver "T: 132
body:
- '"IO1":255,"IO2":255'" 
rosservice call /driver -r 10 "T: 132
body:
- '"IO1":255,"IO2":255'" 
rosservice call /driver -r 10 "T: 132
body:
- '"IO1":255,"IO2":255'" 
rosservice call /driver "T: 132
body:
- '"IO1":255,"IO2":255'" 
jobs -l
ps aux
aux ps
ps aux
kill -9 16776
rosservice call /driver "T: 132
body:
- '"IO1":255,"IO2":255'" 
rosservice call /driver "T: 132
body:
- 'IO1:255','IO2:255'" 
rosservice call /driver "T: 132
body:
- 'IO1:255' 'IO2:255'" 
rosservice call /driver "T: 132
body:
- ["IO1:255" "IO2:255"]" 
rosservice call /driver "T: 132
body:
- ['IO1:255' 'IO2:255']" 
cms
rosservice call /driver "params:
- ''" 
rosservice call /driver "params:
- 'T:600'" 
rosservice call /driver "params:
- 'T:600 L:500'" 
rosservice call /driver "params:
- '[T:600 L:500]'" 
rosservice call /driver "params:
- 'T:600' 'L:500'" 
rosservice call /driver "params:
- ['T:600' 'L:500']" 
rosservice call /driver "T: 132
params: ['IO1:255', 'IO2:255']"
rosservice call /driver "params: ['IO1:255', 'IO2:255']"
rosservice call /driver "params: ['IO1:255' 'IO2:255']"
rosservice call /driver "params:
- 'T:600'" 
rosservice call /driver "params:
- 'T:600', 'L:500" 
rosservice call /driver "params:
- 'T:600', 'L:500'" 
rosservice call /driver "params:
- 'T:600' 'L:500'" 
rosservice call /driver "params:
- ['T:600' 'L:500']" 
rosservice call /driver "params:
- ['T:600', 'L:500']" 
rosservice call /driver "params: ['T:600', 'L:500']" 
rosservice call /driver "params:
- ''" 
clear
rosservice call /driver "T:600 params: ['IO1:255', 'IO2:255']"
rosservice call /driver "T:600"
rosservice call /driver "T:600, params: ['IO1:255', 'IO2:255']"
rosservice call /driver "T: 600
params:
- ''" 
rosservice call /driver "T: 600
params: ['IO1:255']" 
rosservice call /driver "T: 600
params: ['IO1:255', 'IO2:255]" 
rosservice call /driver "T: 600
params: ['IO1:255', 'IO2:255]'" 
clear
rosservice call /driver "T: 600
params: ['IO1:255', 'IO2:255']" 
exit
