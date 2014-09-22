#!/usr/bin/env python

'''
 @author: Adrian Clarke
 @contact: aac2@nist.gov
 @organization: NIST
'''

import sys
import copy
import rospy
import math
from brics_actuator.msg import JointPositions, JointValue
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler #, quaternion_about_axis, quaternion_from_matrix, rotation_matrix
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import control_msgs.msg
import actionlib
from std_msgs.msg import String

''' 
NOTES:
ROBOT 1 IS CLOSER TO THE COMPUTER (LEFT)
ROBOT 2 IS FAR FROM THE COMPUTER (RIGHT)
Main function that does the routine is called "youbot_exec" and is towards the bottom of this document.
'''

# Globally define commonly used stuff:

arm_1_topic_name = "arm_1/arm_controller/position_command"
arm_1_msg_type = JointPositions
grip_1_topic_name = "arm_1/gripper_controller/position_command"
grip_1_msg_type = JointPositions
arm_2_topic_name = "arm_2/arm_controller/position_command"
arm_2_msg_type = JointPositions
grip_2_topic_name = "arm_2/gripper_controller/position_command"
grip_2_msg_type = JointPositions
joint_uri_1 = ['arm_joint_1','arm_joint_2','arm_joint_3','arm_joint_4','arm_joint_5','gripper_finger_joint_l','gripper_finger_joint_r']
joint_uri_2 = ['arm_2_joint_1','arm_2_joint_2','arm_2_joint_3','arm_2_joint_4','arm_2_joint_5','gripper_2_finger_joint_l','gripper_2_finger_joint_r']

# Turn a desired gripper opening into a brics_actuator-friendly message
def make_grip_msg(opening_mm, joint_uri):
    
    left = opening_mm/2000.
    right = opening_mm/2000.
    # create joint positions message
    jp = JointPositions()
    
    # create joint values message for both left and right fingers
    jvl = JointValue()
    jvr = JointValue()
    
    # Fill in the gripper positions desired
    # This is open position (max opening 0.0115 m)
    jvl.joint_uri = joint_uri[5]
    jvl.unit = 'm'
    jvl.value = left
    jvr.joint_uri = joint_uri[6]
    jvr.unit = 'm'
    jvr.value = right
     
    # Append those onto JointPositions
    jp.positions.append(copy.deepcopy(jvl))
    jp.positions.append(copy.deepcopy(jvr))
    
    return jp

# Turn a desired gripper opening into a brics_actuator-friendly message
def make_arm_msg(arm_js, joint_uri):
    
    A1 = arm_js[0]
    A2 = arm_js[1]
    A3 = arm_js[2]
    A4 = arm_js[3]
    A5 = arm_js[4] 
    # create joint positions message
    jp = JointPositions()
    # create joint values message for all the joints
    jv1 = JointValue()
    jv2 = JointValue()
    jv3 = JointValue()
    jv4 = JointValue()
    jv5 = JointValue()
    # Fill in the arm positions. 
    jv1.joint_uri = joint_uri[0]
    jv1.unit = 'rad'
    jv1.value = A1
    jv2.joint_uri = joint_uri[1]
    jv2.unit = 'rad'
    jv2.value = A2
    jv3.joint_uri = joint_uri[2]
    jv3.unit = 'rad'
    jv3.value = A3
    jv4.joint_uri = joint_uri[3]
    jv4.unit = 'rad'
    jv4.value = A4
    jv5.joint_uri = joint_uri[4]
    jv5.unit = 'rad'
    jv5.value = A5
    # Append those onto JointPositions
    jp.positions.append(copy.deepcopy(jv1))
    jp.positions.append(copy.deepcopy(jv2))
    jp.positions.append(copy.deepcopy(jv3))
    jp.positions.append(copy.deepcopy(jv4))
    jp.positions.append(copy.deepcopy(jv5))
    
    return jp

# Fundamental movements: move grippers and arms using brics_actuator publishing
def move_gripper_1(opening_mm):
  # Create a publisher to gripper controller
  gripper_1_command_publisher = rospy.Publisher(grip_1_topic_name, grip_1_msg_type, queue_size = 5)

  # setup the loop rate for the node
  r = rospy.Rate(1) # 10hz
  
  # Fill in your desired opening.
  # Make sure width of opening is no larger than 0.023 m (23 mm)
  jp = make_grip_msg(opening_mm, joint_uri_1)
  # Initialize the timer for gripper publisher
  time = rospy.get_rostime()
  t0 = time.secs
  while not rospy.is_shutdown():
      rospy.loginfo("moving gripper")
      gripper_1_command_publisher.publish(jp)
      r.sleep()
      time = rospy.get_rostime() 
      t1 = time.secs
      if (t1-t0) > 1:
        break

def move_gripper_2(opening_mm):
  # Create a publisher to gripper controller
  gripper_2_command_publisher = rospy.Publisher(grip_2_topic_name, grip_2_msg_type, queue_size = 5)

  # setup the loop rate for the node
  r = rospy.Rate(1) # 10hz
  
  # Fill in your desired opening.
  # Make sure width of opening is no larger than 0.023 m (23 mm)
  jp = make_grip_msg(opening_mm, joint_uri_2)
  # Initialize the timer for gripper publisher
  time = rospy.get_rostime()
  t0 = time.secs
  while not rospy.is_shutdown():
      rospy.loginfo("moving gripper")
      gripper_2_command_publisher.publish(jp)
      r.sleep()
      time = rospy.get_rostime() 
      t1 = time.secs
      if (t1-t0) > 1:
        break

def move_grippers(opening_1_mm, opening_2_mm):
  # Create a publisher to gripper controller
  gripper_1_command_publisher = rospy.Publisher(grip_1_topic_name, grip_1_msg_type, queue_size = 5)
  gripper_2_command_publisher = rospy.Publisher(grip_2_topic_name, grip_2_msg_type, queue_size = 5) 
  # setup the loop rate for the node
  r = rospy.Rate(1) # 10hz
  
  # Fill in your desired opening.
  # Make sure width of opening is no larger than 0.023 m (23 mm)
  jp1 = make_grip_msg(opening_1_mm, joint_uri_1)
  jp2 = make_grip_msg(opening_2_mm, joint_uri_2)
  # Initialize the timer for gripper publisher
  time = rospy.get_rostime()
  t0 = time.secs
  while not rospy.is_shutdown():
      rospy.loginfo("moving gripper")
      gripper_1_command_publisher.publish(jp1)
      gripper_2_command_publisher.publish(jp2)
      r.sleep()
      time = rospy.get_rostime() 
      t1 = time.secs
      if (t1-t0) > 1:
        break

def move_arm_1(cmd):
  # Create a publisher to arm controller
  arm_command_publisher = rospy.Publisher(arm_1_topic_name, arm_1_msg_type, queue_size = 5)

  # setup the loop rate for the node
  r = rospy.Rate(1) # 10hz
  
  # Make sure you put in valid joint states or bad things happen
  arm_cmd = make_arm_msg(cmd, joint_uri_1)
  # Initialize the timer for arm publisher
  time = rospy.get_rostime()
  t0 = time.secs
  while not rospy.is_shutdown():
      rospy.loginfo("moving arm")
      arm_command_publisher.publish(arm_cmd)
      r.sleep()
      time = rospy.get_rostime() 
      t1 = time.secs
      if (t1-t0) > 1:
        break

def move_arm_2(cmd):
  # Create a publisher to arm controller
  arm_command_publisher = rospy.Publisher(arm_2_topic_name, arm_2_msg_type, queue_size = 5)

  # setup the loop rate for the node
  r = rospy.Rate(1) # 10hz
  
  # Make sure you put in valid joint states or bad things happen
  arm_cmd = make_arm_msg(cmd, joint_uri_2)
  # Initialize the timer for arm publisher
  time = rospy.get_rostime()
  t0 = time.secs
  while not rospy.is_shutdown():
      rospy.loginfo("moving arm")
      arm_command_publisher.publish(arm_cmd)
      r.sleep()
      time = rospy.get_rostime() 
      t1 = time.secs
      if (t1-t0) > 1:
        break

def move_arms(cmd_1, cmd_2):
  # Create a publisher to arm controller
  arm_1_command_publisher = rospy.Publisher(arm_1_topic_name, arm_1_msg_type, queue_size = 5)
  arm_2_command_publisher = rospy.Publisher(arm_2_topic_name, arm_2_msg_type, queue_size = 5)
  # setup the loop rate for the node
  r = rospy.Rate(1) # 10hz
  
  # Make sure you put in valid joint states or bad things happen
  arm_1_cmd = make_arm_msg(cmd_1, joint_uri_1)
  arm_2_cmd = make_arm_msg(cmd_2, joint_uri_2)
  # Initialize the timer for arm publisher
  time = rospy.get_rostime()
  t0 = time.secs
  while not rospy.is_shutdown():
      rospy.loginfo("moving arms")
      arm_1_command_publisher.publish(arm_1_cmd)
      arm_2_command_publisher.publish(arm_2_cmd)
      r.sleep()
      time = rospy.get_rostime() 
      t1 = time.secs
      if (t1-t0) > 1:
        break


# ***MAIN FUNCTION***
def youbot_exec(arm_1_topic_name, arm_1_msg_type, grip_1_topic_name, grip_1_msg_type, arm_2_topic_name, arm_2_msg_type, grip_2_topic_name, grip_2_msg_type):

  # Initialize rospy
  rospy.init_node('youbot_exec',
                  anonymous=False)

  # ====== Command List. Give Every Command a Unique Name ====== #

  # Fill in command SPECIAL with joint positions to test poses. Comment out all the poses that you don't want to use in the main routine.
  
  # Robot 1 is the LEFT HAND one close to computer.
  # Robot 2 is the RIGHT HAND one far from computer.
  prepick_rob2 = [2.945,1.5,-1.62,3.13,2.94]  
  pick_rob2 = [2.945,1.89,-1.62,3.13,4.5]  
  cand = [2.94,1.0,-2.1,1.8,2.94] # candle
  trans1 = [1.4,0.71,-2.0,3.3,2.9489] # Rob1 prepares for giving it to Rob2
  pretrans2 = [4.0,1.35,-1.94,2.4,1.39] # Rob2 gets ready to receive the ball
  trans2 = [4.5,1.35,-1.94,2.41,1.39] # Rob2 takes ball from Rob1 
  posttrans2 = [4.5,0.9,-1.94,2.4,1.39] # Rob2 moves out the way
  premach2 = [1.93, 0.55, -1.3, 2.5, 2.95] # Machine 2 pre spot, it is right after loading point Rob2 uses it
  mach2 = [1.93, 1.05, -1.3, 3.16, 2.95] # Machine 2 spot, it is right after loading point. Rob2 uses it
  preload = [1.37, 1.15, -1.7, 2.83, 2.95] # Loading pre point. Rob 2 uses it
  load = [1.37, 1.55, -1.7, 2.83, 2.95] # Loading point. Rob 2 uses it
  premach1 = [2.22, 0.35, -2.34, 0.92, 2.95] # Machine 1 pre spot. Rob1 uses it
  mach1 = [2.22, 0.05, -2.34, 0.92, 2.95] # Machine 1 spot. Rob1 uses it
  prepick_rob1 = [2.96, 1.36, -1.62, 3.07, 2.94] # Robot 1 is the left hand one. This is the pre pickup spot 
  pick_rob1 = [2.96, 1.96, -1.62, 3.07, 2.94] # Robot 1 is the left hand one. This is the pickup spot 
  ready_tight_spot = [3.8, 1.2, -0.02, 0.8, 2.93] # PRE-PRE tight spot, gets ready for *ANY* operation involving the tight spot.
  # IMPORTANT: ready_tight_spot should also be done AFTER operation as well as getting ready before it.
  pretight_spot_place = [3.8, 1.75, -0.7, 0.85, 2.929] # Pre tight spot for placing. Note identical to post-tight-spot-place.
  # IMPORTANT: picking up from the tight spot doesn't require a pretight_spot!!
  tight_spot = [3.8, 1.85, -0.9, 0.96, 2.929] # tight spot
  posttight_spot_pickA = [3.8, 1.85, -0.9, 0.90, 2.929] # First post tight spot after picking (lifts it a bit)
  posttight_spot_pickB = [4.5, 1.85, -0.9, 0.90, 2.929] # Second post tight spot after picking (clears the spot)
  posttight_spot_place = [3.8, 1.75, -0.7, 0.85, 2.929] # Post tight spot for placing. Note identical to pre-tight-spot-place
  opening = 22 # opening gap in mm
  closing = 9 # closing gap in mm. DO NOT REDUCE BELOW 8 mm! Gripper finger(s) may become out of place from biting too hard.


  SPECIAL = [3.8, 1.75, -0.7, 0.85, 2.929] # USE FOR TESTING

  # ======== MAIN ROUTINE. Send a series of gripper and arm motions. ======== #
  
  # Ctrl-f to look at/modify these functions: 
  # move_arms - both arms move roughly at the same time
  # move_grippers - both grippers move roughly at the same time
  # move_gripper_1 - move rob1's gripper only
  # move_gripper_2 - move rob2's gripper only
  # move_arm_1 - move rob1's arm only
  # move_arm_2 - move rob2's arm only
  # future implementation: Move grippers and arms simultaneously? i.e. gripper1+arm2, gripper1+arm1, gripper2+arm1, etc

  while not rospy.is_shutdown(): 
      # Test setup. Starts with Candle, so make sure nothing is in the way of it getting to candle at startup
      # because it can be quite violent.
      move_grippers(opening, opening)       
      move_arms(cand, cand)
      move_arms(premach1, preload)
      move_arms(mach1, load)  
      move_grippers(closing, closing)
      move_arms(premach1, preload) 
      move_arms(cand, premach2)
      move_arms(cand, mach2)
      move_gripper_2(opening)
      move_arms(trans1, premach2)
      move_arm_2(cand)
      move_arm_2(pretrans2)
      move_arm_2(trans2)
      move_gripper_2(closing)
      move_gripper_1(opening)
      move_arm_2(posttrans2)
      move_arms(cand, cand)
      move_arms(ready_tight_spot,preload)
      move_arms(tight_spot,load)
      move_grippers(closing,opening)
      move_arms(posttight_spot_pickA, preload)
      move_arms(posttight_spot_pickB, premach2)
      move_arms(premach1, mach2)
      move_gripper_2(closing)
      move_arms(mach1, premach2)
      move_gripper_1(opening)
      move_arms(cand, prepick_rob2)
      move_arms(prepick_rob1, pick_rob2)
      move_gripper_2(opening)
      rospy.sleep(1.5) # This is just to wait for the ball to roll. Can be longer if necessary.
      move_arms(pick_rob1, prepick_rob2)
      move_gripper_1(closing)
      move_arm_1(ready_tight_spot)
      move_arm_1(pretight_spot_place)
      move_arm_1(tight_spot)
      move_gripper_1(opening)
      move_arm_1(posttight_spot_place)
      move_arm_1(ready_tight_spot)
      move_arms(cand,cand)
      n = raw_input("""
Restart the routine?(Y/N)
WARNING! If you pressed ctrl-C to get to this message, you cannot restart the routine
due to rospy being shutdown. It will fail. Type N to exit if this is the case.""")
      if n.strip() == 'n': 
        break
      elif n.strip() == 'N':
        break
      elif n.strip() == 'y' or n.strip() == 'Y':
        continue
      else:
        print "Invalid input. Exiting..."
        break

# This is the main subroutine

if __name__=='__main__':
  try:  
    youbot_exec(arm_1_topic_name,
				arm_1_msg_type,
				grip_1_topic_name,
				grip_1_msg_type,
				arm_2_topic_name,
				arm_2_msg_type,
				grip_2_topic_name,
				grip_2_msg_type)
  except rospy.ROSInterruptException:
    pass
