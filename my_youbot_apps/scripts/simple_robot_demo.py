#!/usr/bin/env python

'''
 @author: Rick Candell, Adrian Clarke
 @contact: rick.candell@nist.gov
 @organization: NIST
 @license: Public Domain
'''

import sys
import copy
import rospy
import math
from tf.transformations import quaternion_from_euler #, quaternion_about_axis, quaternion_from_matrix, rotation_matrix
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import control_msgs.msg
import actionlib
from brics_actuator.msg import JointPositions, JointValue
  
def jtg_feedback_cb(data):  
  rospy.loginfo('arm is moving')
  rospy.loginfo(data)
  
def trajectory_toffset(traj, tsecs):
  for ii in range(0,len(traj.points)):
    traj.points[ii].time_from_start.secs+= tsecs
  return traj

'''
def massage_traj(traj_in, tstep):
  traj_out = copy.deepcopy(traj_in)
  traj_out.joint_names = traj_in.joint_names
  npts = len(traj_in.points)
  for ii in range(npts):
    traj_out.points[ii].velocities = [0,0,0,0,0] 
    #traj_out.points[ii].accelerations = [0,0,0,0,0]   
    traj_out.points[ii].time_from_start = rospy.Duration(float(ii)*tstep)
  return traj_out
'''

def massage_traj(traj_in):
  traj_out = JointTrajectory()
  traj_out.joint_names = traj_in.joint_names
  npts = len(traj_in.points)

  # take first point
  traj_out.points.append(copy.deepcopy(traj_in.points[0]))
  traj_out.points[0].velocities = [0,0,0,0,0]
  traj_out.points[0].accelerations = [0,0,0,0,0]

  # take middle point
#  midpt = int(npts/2)
#  traj_out.points.append(copy.deepcopy(traj_in.points[midpt]))
#  traj_out.points[1].velocities = [0,0,0,0,0]
#  traj_out.points[1].accelerations = [0,0,0,0,0]

  # take last point
  traj_out.points.append(copy.deepcopy(traj_in.points[-1]))
  traj_out.points[-1].velocities = [0,0,0,0,0]
  traj_out.points[-1].accelerations = [0,0,0,0,0]

  print "Points in traj: " + str(len(traj_out.points))
  return traj_out

# Turn a desired gripper opening into a brics-friendly message
def make_brics_msg(opening_m):
    
    left = opening_m/2.
    right = opening_m/2.
    # create joint positions message
    jp = JointPositions()
    
    # create joint values message for both left and right fingers
    jvl = JointValue()
    jvr = JointValue()
    
    # Fill in the gripper positions desired
    # This is open position (max opening 0.0115 m)
    jvl.joint_uri = 'gripper_finger_joint_l'
    jvl.unit = 'm'
    jvl.value = left
    jvr.joint_uri = 'gripper_finger_joint_r'
    jvr.unit = 'm'
    jvr.value = right
     
    # Append those onto JointPositions
    jp.positions.append(copy.deepcopy(jvl))
    jp.positions.append(copy.deepcopy(jvr))
    
    return jp

def move_gripper(grip_topic_name, grip_msg_type, opening_m):

  # Create a publisher to gazebo arm controller
  gripper_command_publisher = rospy.Publisher(grip_topic_name, grip_msg_type, latch=True)

  # setup the loop rate for the node
 # r = rospy.Rate(10) # 10hz
  

  # Fill in your desired opening.
  # Make sure width of opening is no larger than 0.023 m (23 mm)
  jp = make_brics_msg(opening_m)
  print jp
  # Initialize the timer for gripper publisher
  t0 = rospy.get_rostime
  while not rospy.is_shutdown():
      rospy.loginfo("moving gripper")
      gripper_command_publisher.publish(jp) 
      #r.sleep()
      t1 = rospy.get_rostime
      if t1 > 8:
        break

def move_arm(client, group, nextPoseSet, end_effector_link):

  result_code = None
  group.clear_pose_targets()
  group.set_pose_targets(nextPoseSet, end_effector_link)
  #group.set_goal_position_tolerance(0.01)
  #group.set_goal_orientation_tolerance(0.1)     
  
  '''
  Call Moveit! planner to produce a trajectory
  '''
  plan = group.plan()

  if len(plan.joint_trajectory.points) == 0:
    rospy.loginfo("Trajectory not found for given pose and solver")
    #break  # We exit if a plan is not found
  else:
    plan.joint_trajectory = trajectory_toffset(plan.joint_trajectory, 0)
    rospy.loginfo("Moving to pose ")

    #print "Trajectory computed"
    #rospy.loginfo(plan)
  
    # Creates a goal to send to the action server.
    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    goal.trajectory = copy.deepcopy(plan.joint_trajectory)
    goal.trajectory = massage_traj(goal.trajectory)

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=jtg_feedback_cb)
  
    # Waits for the server to finish performing the action.
    client.wait_for_result()  
  
    # get the result code
    result_code = client.get_result()

  return result_code
  
  
def JointTrajectory_client(arm_action_server_name, gripper_action_server_name):
  '''
  Main MoveIt! publisher demo function
  '''
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('robot_exec',
                  anonymous=False)

  ## Instantiate a RobotCommander object. This object is an interface to
  ## the robot as a whole. 
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object. This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object. This object is an interface
  ## to one group of joints. In this case the group is the joints in the left
  ## arm. This interface can be used to plan and execute motions on the left
  ## arm.
  print "Creating move group commander object"
  group = moveit_commander.MoveGroupCommander("manipulator")
  group.set_planning_time(15)
  #print "Current pose"
  #print group.get_current_pose()
  #print "Current rpy"
  #print group.get_current_rpy()
  #return None

  ## Create a client for the action server
  rospy.loginfo("Create arm joint trajectory action client " + arm_action_server_name)
  arm_client = actionlib.SimpleActionClient(arm_action_server_name, 
                                        control_msgs.msg.FollowJointTrajectoryAction)
  
  rospy.loginfo("Create gripper joint trajectory action client " + gripper_action_server_name)
  #gripper_client = actionlib.SimpleActionClient(gripper_action_server_name, 
  #                                      control_msgs.msg.FollowJointTrajectoryAction)
  
  # Waits until the action server has started up and started
  # listening for goals.
  rospy.loginfo("waiting for arm action server")
  arm_client.wait_for_server()  
  #rospy.loginfo("waiting for gripper action server")
  #gripper_client.wait_for_server()

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Planning reference frame: %s" % group.get_planning_frame()
  
  ## We can also print the name of the end-effector link for this group
  print "============ End effector link: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups: %s" % robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  #print "============ Printing robot state"
  #print robot.get_current_state()
  #print "============"
  
  '''
  This section builds a set of waypoints.  The joint trajectory (the plan) will be
  computed using these waypoints.
  '''
  # POSE1
  end_effector_link = "gripper_pointer_link"
  roll = 0
  pitch = 0 #math.pi
  yaw = 0 #math.pi/4
  quat = quaternion_from_euler(roll,pitch,yaw)
  P = (0.0,0.0,0.55)
  pose1 = geometry_msgs.msg.PoseStamped()
  pose1.header.frame_id = "base_link"
  pose1.pose.position.x = P[0]
  pose1.pose.position.y = P[1]
  pose1.pose.position.z = P[2]
  pose1.pose.orientation.x = quat[0]
  pose1.pose.orientation.y = quat[1]
  pose1.pose.orientation.z = quat[2]
  pose1.pose.orientation.w = quat[3]  
  pose1_gripper_pos = 0.02

  # POSE 2
  roll = 0
  pitch = -math.pi
  yaw = math.pi/4
  quat = quaternion_from_euler(roll,pitch,yaw)
  P = (0.2,-0.18,0.1)
  pose2 = geometry_msgs.msg.PoseStamped()
  pose2.header.frame_id = "base_link"
  pose2.pose.position.x = P[0]
  pose2.pose.position.y = P[1]
  pose2.pose.position.z = P[2]
  pose2.pose.orientation.x = quat[0]
  pose2.pose.orientation.y = quat[1]
  pose2.pose.orientation.z = quat[2]
  pose2.pose.orientation.w = quat[3]
  pose2_gripper_pos = 0.0
    
  # POSE 3
  #pose3 = copy.deepcopy(pose2)
  #pose3.pose.position.y = -pose3.pose.position.y
  #pose3_gripper_pos = 0.02
  roll = 0
  theta = -math.pi
  yaw = math.pi/4
  quat = quaternion_from_euler(roll,pitch,yaw)
  P = (0.2,0.18,0.1)
  pose3 = geometry_msgs.msg.PoseStamped()
  pose3.header.frame_id = "base_link"
  pose3.pose.position.x = P[0]
  pose3.pose.position.y = P[1]
  pose3.pose.position.z = P[2]
  pose3.pose.orientation.x = quat[0]
  pose3.pose.orientation.y = quat[1]
  pose3.pose.orientation.z = quat[2]
  pose3.pose.orientation.w = quat[3]
  pose3_gripper_pos = 0.0
  
  # Pose list
  poses = []
  poses.append(copy.deepcopy(pose1.pose))
  poses.append(copy.deepcopy(pose2.pose))
  poses.append(copy.deepcopy(pose1.pose))
  poses.append(copy.deepcopy(pose3.pose))
  
  # gripper list
  gripper_poses = []
  gripper_poses.append(pose1_gripper_pos)
  gripper_poses.append(pose2_gripper_pos)
  gripper_poses.append(pose1_gripper_pos)
  gripper_poses.append(pose3_gripper_pos)
  
  # Enforce that the poses are specified in reference to the base_link
  group.set_pose_reference_frame("base_link")
  rospy.loginfo("Number of poses in list: " + str(len(poses)))  
  
  '''
  It is possible to overrid the default planner algorithm here
  '''
  #group.set_planner_id('RRTkConfigDefault')
  
  iiPose = 0
  nPoses =  len(poses)
  print "nPoses: " + str(nPoses)
  while not rospy.is_shutdown():
    
    nextPoseSet = []
    
    # determine pose index
    pindex = iiPose % nPoses
    print "Pose: " + str(pindex+1)
    
    # set the next pose
    nextPoseSet.append(poses[pindex])
    rospy.logdebug("Next arm pose")
    rospy.logdebug(nextPoseSet)
    
    # move the arm
    result_code = move_arm(arm_client, group, nextPoseSet, end_effector_link)
    #rospy.loginfo("move arm return code")
    #rospy.loginfo(result_code)  
    
    #opening_m = gripper_poses[pindex] 
    #move_gripper("/arm_1/gripper_controller/position_command", JointPositions, opening_m)
    #rospy.loginfo("move gripper return code")
    #rospy.loginfo("Done")

    # sleep ?
    rospy.sleep(1)
    
    # increment pose counter
    if iiPose > 10:
      break
    else:
      iiPose += 1
  

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
  
  # Exit the MoveIt commander gracefully.
  #  This is a bug work-around.  It's supposed to prevent a core dump upon exiting.
  moveit_commander.os._exit(0)
  
# 
# This is the main subroutine
#  
if __name__=='__main__':
  try:
    #JointTrajectory_publisher(
    #     topic_name="/arm_1/arm_controller/command", 
    #     msg_type=JointTrajectory)     
    JointTrajectory_client('/arm_1/arm_controller/follow_joint_trajectory',
                           '/arm_1/gripper_controller/follow_joint_trajectory')
    
  except rospy.ROSInterruptException:
    moveit_commander.os._exit(0)



