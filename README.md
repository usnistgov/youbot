Package: my\_youbot\_moveit
================
Author: Rick Candell <br>
Contact: rick.candell@nist.gov <br>

# Overview
This package serves to demonstrate how to develop a control executive using the subscribe/publish and simple action client approaches.  The package is designed for the Kuka Youbot.  It contains scripts and data description files required to launch the MoveIt! executable and the Gazebo controllers for the Youbot.  The Youbot robot description (URDF and XACRO) files and controller implementations are provided with the youbot packages not included in the *my\_youbot\_moveit* package.  This package contains python scripts that demonstrate how to use the MoveIt! library and Gazebo controllers.

# Package Contents
This package was created using the MoveIt! Setup Assistant.  Most launch files and config files were created using this tool.  The contents of this package are described in the following sections.

## config directory
This folder contains all of the confguration files necessary for the Gazebo controller plugins and the MoveIt! path planner.  Kinematic chains (i.e., move groups) are defined in the youbot.srdf file.

## launch directory
Most files in this directory were create by the MoveIt! setup assistant; however, files prefixed with *my_* are necessary to run the *my_youbot_moveit* software applications.

Launch File  | Description
------------- | -------------
my_gazebo_arm_with_moveit.launch  | This file is used to launch Gazebo with controller plugin, the MoveIt! node, and RViz if running Gazebo headless


## scripts directory
This folder contains all of the scripts required to run the software.

- **my\_youbot\_driver.py** is a legacy script that was used to demonstrate control of the robot using the simple susbscribe/publish method. 

- **my_youbot\_gazebo\_exec.py** executes a simple action client.  The Gazebo controller plugin implements the action server.  

The action client is the preferred method for the execution of joint trajectories.  The subscribe/publish method is possible but is not recommended.


# Installation
- Install Ubuntu 12.04 [http://releases.ubuntu.com/12.04/](http://releases.ubuntu.com/12.04/)

- Install ROS Hydro [http://wiki.ros.org/hydro/Installation/Ubuntu](http://wiki.ros.org/hydro/Installation/Ubuntu)

- Install ROS dependencies

This list is an attempt to cover all of the dependencies required to run the python scripts and launch files included in this package.  If a package has been overlooked, please notify me through email.

```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-hydro-youbot* 
sudo apt-get install ros-hydro-controller-* ros-hydro-ros-control* 
sudo apt-get install ros-hydro-gazebo-* ros-hydro-joint-state-* ros-hydro-joint-trajectory-*
sudo apt-get install ros-hydro-moveit-full

```

-
- Create and configure your catkin workspace. [http://wiki.ros.org/catkin/Tutorials](http://wiki.ros.org/catkin/Tutorials)
- Install this package under your Catkin workspace. 
```
cd ~/catkin_ws/src
git clone https://github.com/NIST-ICS-SEC-TB/my_youbot_moveit.git
```

# Running the Scipts

## Using Gazebo
Open a terminal and execute the following command.  This will launch gazebo and moveit.  

```
roslaunch my_youbot_moveit my_gazebo_arm_with_moveit.launch
```

Open a new terminal and run:

```
rosrun my_youbot_moveit my_youbot_gazebo_exec.py
```

This will execute a series of poses specified in the python code.  This script will only work with Gazebo.  Some modification is required for interoperabililty with the Youbot hardware.

## Running the action client with the youbot

To be documented later


