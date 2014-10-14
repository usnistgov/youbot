Package: youbot
================
Author: Rick Candell <br>
Contact: rick.candell@nist.gov <br>
License: Public Domain

# Overview
This repository contains many different ROS packages required to run the youbot in simulation and with the actual robot.

# Package Contents

- **youbot_description**
This package contains all of the URDF and XACRO files that describe the build and kinematics of the robot.  Some minor changes have been made from the original debian release.

- **youbot_moveit**
This package was created using the MoveIt! Setup Assistant.  

- **youbot_msgs**
This package contains custom message definitions used by the youbot applications.

- **youbot_apps**
This package contains scripts and launch files to run the robot under gazebo or with the hardware.  This package also contains a core python package for controlling multiple youbots using modules in the *robot_proxy* package. 

- **twoarm_cage**
This package contains all scripts, launch files, and configuration required to run a two arm collaborative experimental platform in the NIST security lab.  Robots under this configuraiton are controlled using independent PC's with a remote ROS master.  Three computers are typically required under this configuration; although, with minor modifications, a single computer with a two-arm driver could be used.

# Installation
- Install Ubuntu 12.04 [http://releases.ubuntu.com/12.04/](http://releases.ubuntu.com/12.04/)

- Install ROS Hydro [http://wiki.ros.org/hydro/Installation/Ubuntu](http://wiki.ros.org/hydro/Installation/Ubuntu)

- Install ROS dependencies:  This list is an attempt to cover all of the dependencies required to run the python scripts and launch files included in this package.  If a package has been overlooked, please notify me through email.

```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-hydro-youbot* ros-hydro-controller-* ros-hydro-ros-control* ros-hydro-gazebo-* ros-hydro-joint-state-* ros-hydro-joint-trajectory-*  ros-hydro-moveit-full
```

- Create and configure your catkin workspace. [http://wiki.ros.org/catkin/Tutorials](http://wiki.ros.org/catkin/Tutorials)
- Install this package under your Catkin workspace. 

```
cd ~/catkin_ws/src
git clone [this-repository-url]
```

# Running the Apps

## Using Gazebo
Open a terminal and execute the following command.  This will launch gazebo and moveit.  

```
roslaunch youbot_apps gazebo.launch
```

Open a new terminal and run:

```
roslaunch youbot_apps moveit.launch
```

Open a new terminal and run:

```
rosrun youbot_apps youbot_gazebo_exec.py
```

This will execute a series of poses specified in the python code.  This script will only work with Gazebo.  Some modification is required for interoperabililty with the Youbot hardware.

## Youbot Robot (not Gazebo)
This is similar to the Gazebo configuration, but the internal messages are very different.  The major difference between the gazebo implementation and the youbot implementation is that the Gazebo configuration uses the action server which employs a torque controller behind the scenes.  Accurate trajectory following is possible with the gazebo model.  Accuracy of the model to the real-world has yet to be ascertained.

The youbot-based controller uses the brics_actuator messages and a position/velocity controller hardware module in the youbot base link.  A torque controller has not been developed by Kuka as of October 2014.  Accuracy or repeatability testing would require a torque controller.

To run the demonstrations in the youbot apps, follow these instructions.

```
sudo bash
roslaunch youbot_apps youbot_one_arm.launch
```

or if using two arms

```
sudo
roslaunch youbot_apps youbot_two_arm.launch
```

Then in another terminal

```
roslaunch youbot_apps moveit.launch
```

Then in yet another terminal

```
roslaunch youbot_apps circle_tap.launch
```

##Running the Two Arm Cage
This is a little more complex and requires careful configuration of the user environment.  Look for the README file under the twoarm_cage package.



