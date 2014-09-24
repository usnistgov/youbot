#!/bin/bash

# run the roscore terminal
#x-terminal-emulator --tab -e "bash -c 'roscore && sleep 5'" 

# youbot driver
# note that the environment must be configured for an interactice shell  
#  that supports ros.  Do this by adding ros environment to /etc/profile.d
gnome-terminal -e "sudo -i bash -c 'roslaunch youbot_apps youbot_two_arm.launch'" &
read -p "Press [Enter] key to start backup..."

# run the moveit terminal
roslaunch youbot_apps moveit.launch &
#gnome-terminal -e 'roslaunch youbot_apps moveit.launch' &
#sleep 5

# run the controller exec for arm 1 and 2
rosrun youbot_apps two_arm_collab_demo.py &
#gnome-terminal -e 'rosrun youbot_apps two_arm_collab_demo.py' &





