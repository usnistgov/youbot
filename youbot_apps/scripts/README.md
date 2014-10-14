## Two-Arm Collaborative Demo with Single Driver
================
Author: Rick Candell <br>
Contact: rick.candell@nist.gov <br>
License: Public Domain

## Notes
The two-arm collaborative demo is designed to execute on one machine with a single two-arm driver instance and one application instance.

To launch the demo, run:

In one terminal, launch the two-arm driver:

'''
roslaunch youbot_apps youbot_two_arm.launch
'''

In another terminal, launch moveit

'''
roslaunch youbot_apps moveit.launch
'''

In another terminal, launch the application

'''
rosrun youbot_apps two_arm_collab_demo.py
'''

