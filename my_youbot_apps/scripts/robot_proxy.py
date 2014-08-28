#!/usr/bin/env python

'''
@author: Rick Candell
@organization: NIST
@contact: rick.candell@nist.gov
@license: MIT
'''

import sys
import copy
import rospy
import math
from abc import ABCMeta, abstractmethod
from brics_actuator.msg import JointPositions, JointValue
from tf.transformations import quaternion_from_euler 
#import moveit_commander
#import moveit_msgs.msg
#import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import control_msgs.msg
import actionlib
from my_youbot_msgs.srv import JointTrajectoryRequest

class YoubotProxy(object):
  
  _arm_joint_names = ['arm_joint_1','arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
  _gripper_joint_names = ['gripper_finger_joint_l', 'gripper_finger_joint_r']
                  
  def __init__(self, robot_id, topic, msg_type):
    self._robot_id = robot_id
    self._robot_proxy_name = "robot_proxy_" + str(self._robot_id)
    self._pub = rospy.Publisher(topic, msg_type, queue_size=10)
    rospy.init_node(self._robot_proxy_name, anonymous=False)    
    rospy.loginfo("proxy node initialized")
    rospy.loginfo(self._robot_proxy_name)   
    
    @abstractmethod
    def actuate_arm(self):
        pass    

    @abstractmethod
    def actuate_gripper(self):
        pass      
      
      
class YoubotGazeboProxy(YoubotProxy):
  
  _rate = 50
   
  def __init__(self, robot_id, topic, msg_type):   
    
    # initialize member variables 
    self._arm_server_name = "/arm_" + str(robot_id) + "/arm_controller/follow_joint_trajectory"
    self._arm_server_msg_type = control_msgs.msg.FollowJointTrajectoryAction
    self._gripper_server_name = "/arm_" + str(robot_id) + "/arm_controller/follow_joint_trajectory"
    self._gripper_server_msg_type = control_msgs.msg.FollowJointTrajectoryAction  
    self._command_service_root = "CommandRobot"    
    
    # initialize this ros node
    super(YoubotGazeboProxy,self).__init__(robot_id, topic, msg_type)
    rospy.loginfo("proxy node of type gazebo")
    
    # create the client connection to the arm action server
    self._arm_client = actionlib.SimpleActionClient(self._arm_server_name, self._arm_server_msg_type)
    rospy.loginfo("waiting for arm action server")
    self._arm_client.wait_for_server()
    rospy.loginfo("connected to arm action server")
    
    # create the service to accept instructions
    self._command_server = rospy.Service(self._command_service_root + "_" + str(self._robot_id), JointTrajectoryRequest, self.execute_goal_cb)
    rospy.loginfo("robot command service ready: " + self._command_server.resolved_name)
    rospy.loginfo("robot ready for commands from supervisor")

    
  def execute_goal_cb(self, req):   
    '''
    @param goal: FollowJointTrajectoryGoal object  
    '''       
    result_code = 0
    result_code = self.actuate_gripper(req.premove_gripper_opening_mm)
    result_code = self.actuate_arm(req.positions)
    result_code = self.actuate_gripper(req.postmove_gripper_opening_mm)
    
    return result_code

  def actuate_arm(self, positions):
    '''
    This function will move the arm to the current joint coordinates
    @param positions: a JointTrajectory object
    '''
    result_code = 0
    for ii in range(positions.points):
      
      # this point in trajectory
      point = positions.points[ii]
      
      # Creates a goal to send to the action server.
      goal = control_msgs.msg.FollowJointTrajectoryGoal()
      goal.trajectory.points.append(point)

      # Sends the goal to the action server.
      self._arm_client.send_goal(goal, feedback_cb=self.actuate_arm_fb)
      
      # Waits for the server to finish performing the action.
      self._arm_client.wait_for_result()  
      
      # get the result code
      result_code = self._arm_client.get_result()
          
    return result_code
    
  def actuate_arm_fb(self, fb):
    return None
  
  def actuate_gripper(self, opening_mm):
    result_code = 0
    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = self._gripper_joint_names
    pt = JointTrajectoryPoint()
    pt.positions = opening_mm/2.0
    goal.trajectory.points.append(pt)
    return result_code
  
  
# This is the main subroutine
if __name__=='__main__':
  try:  
    print "run proxy test"
    r1g = YoubotGazeboProxy(1, "/arm_1/arm_controller/command", JointTrajectory)
    #r2g = YoubotGazeboProxy(2, "/arm_1/arm_controller/command", JointTrajectory)
  except Exception as e:
    rospy.logerr(e)
    pass    
    
    
    
    
      