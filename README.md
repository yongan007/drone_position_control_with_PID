# drone_position_control_with_PID




## Start PX4 :

make px4_sitl gazebo

## Then start MAVROS:

roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"

## Toluanch velocity control:

This script is testing takeoff and Drone will fly to desired positon and stay in the point:

roslaunch position_fight velocity_control.launch 

## Toluanch position control:

This script is testing takeoff and Drone will fly by follow command such hower, move, land:

roslaunch position_fight position_control.launch 




