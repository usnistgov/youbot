#!/usr/bin/env bash

rostopic pub --once robot_depends youbot_msgs/UpdateDependency "name: '/system/begin'
status: true" 


