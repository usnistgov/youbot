Package: youbot
================
Author: Rick Candell <br>
Contact: rick.candell@nist.gov <br>

# Overview
This repository contains many different ROS packages required to run the youbot in simulation and with the actual robot

# Package Contents

- **youbot_description**
This package contains all of the URDF and XACRO files that describe the build and kinematics of the robot.  Some minor changes have been made from the original debian source.

- **my\_youbot\_moveit**
This package was created using the MoveIt! Setup Assistant.  Most launch files and config files were created using this tool.  The contents of this package are described in the following sections.

- **my_youbot_apps**
This package contains scripts and launch files to run the robot.  

- **my_youbot_planning_server** FUTURE 
This package serves up instructions to the robot(s) and is a work in progress.  No my_youbot_app uses this service as of yet.

- **my_youbot_msgs**
This package contains messages for future use with the planning server

# Installation
- Install Ubuntu 12.04 [http://releases.ubuntu.com/12.04/](http://releases.ubuntu.com/12.04/)

- Install ROS Hydro [http://wiki.ros.org/hydro/Installation/Ubuntu](http://wiki.ros.org/hydro/Installation/Ubuntu)

- Install ROS dependencies:  This list is an attempt to cover all of the dependencies required to run the python scripts and launch files included in this package.  If a package has been overlooked, please notify me through email.

```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-hydro-youbot* 
sudo apt-get install ros-hydro-controller-* ros-hydro-ros-control* 
sudo apt-get install ros-hydro-gazebo-* ros-hydro-joint-state-* ros-hydro-joint-trajectory-*
sudo apt-get install ros-hydro-moveit-full

```

- Create and configure your catkin workspace. [http://wiki.ros.org/catkin/Tutorials](http://wiki.ros.org/catkin/Tutorials)
- Install this package under your Catkin workspace. 

```
cd ~/catkin_ws/src
git clone https://github.com/NIST-ICS-SEC-TB/youbot.git
```

# Running the Apps

## Using Gazebo
Open a terminal and execute the following command.  This will launch gazebo and moveit.  

```
roslaunch my_youbot_apps gazebo.launch
```

Open a new terminal and run:

```
roslaunch my_youbot_apps moveit.launch
```

Open a new terminal and run:

```
rosrun my_youbot_apps my_youbot_gazebo_exec.py
```

This will execute a series of poses specified in the python code.  This script will only work with Gazebo.  Some modification is required for interoperabililty with the Youbot hardware.

## Using the Youbot robot

To be documented later



