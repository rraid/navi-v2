#!/bin/bash

roscore &

rosparam set urg_node/serial_port /dev/ttyACM2
rosrun urg_node urg_node &

roslaunch zed_wrapper zed.launch &

