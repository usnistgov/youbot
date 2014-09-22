#!/usr/bin/env python

# Author Rick Candell

import sys
import copy
import rospy
import math
from tf.transformations import quaternion_from_euler, quaternion_about_axis, quaternion_from_matrix, rotation_matrix
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal, JointTolerance
import actionlib.msg 

from std_msgs.msg import String

def one_good_pose():
  '''
  We know that this is a good pose
  '''
  pose = geometry_msgs.msg.PoseStamped()
  pose.header.seq =0
  pose.header.stamp.secs = 0
  pose.header.stamp.nsecs = 0
  pose.header.frame_id = "base_link"
  pose.pose.position.x = -0.0322978676598
  pose.pose.position.y = 0.0396151681033
  pose.pose.position.z = 0.454002116203
  pose.pose.orientation.x = -0.0995907310894
  pose.pose.orientation.y = 0.0241710355201
  pose.pose.orientation.z = 0.925577199203
  pose.pose.orientation.w = 0.364423236964
  return pose

def get_random_waypoints(move_group=[], n=2):
    ''' 
    Get a series of waypoints with the current pose first
    '''
    geom_points = []
    pose = move_group.get_current_pose().pose
    geom_points.append(copy.deepcopy(pose))
    try:
        for ii in range(0,n):
            pose = move_group.get_random_pose().pose
            geom_points.append(copy.deepcopy(pose))
        return copy.deepcopy(geom_points)    
    except MoveItCommanderException:
        print "get_random_waypoints failed"
        return False
                
def get_home_pose_trajectory():
    '''
    This function will return the joint poses for home position
    '''
    # create joint trajectory message
    jt = JointTrajectory()
    
    # fill the header
    jt.header.seq = 0
    jt.header.stamp.secs = 0 #secs
    jt.header.stamp.nsecs = 0 #nsecs
    jt.header.frame_id = 'base_link'
    
    # specify the joint names
    jt.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
    
    # joint points
    jtp = JointTrajectoryPoint()
    jtp.positions = [0,0,0,0,0]
    jt.points.append(copy.deepcopy(jtp))
    
    return jt
    
                    
def make_trajectory_msg(plan=[], seq=0, secs=0, nsecs=0, dt=2, frame_id='/base_link'):
    '''
    This function will convert the plan to a joint trajectory compatible with the 
    /arm_N/arm_controller/command topic
    '''
    theplan = plan
    
    # create joint trajectory message
    jt = JointTrajectory()
    
    # fill the header
    jt.header.seq = seq
    jt.header.stamp.secs = 0 #secs
    jt.header.stamp.nsecs = 0 #nsecs
    jt.header.frame_id = frame_id
    
    # specify the joint names
    jt.joint_names = theplan.joint_trajectory.joint_names
    
    # fill the trajectory points and computer constraint times
    njtp = len(theplan.joint_trajectory.points)
    for ii in range(0, njtp):
        jtp = JointTrajectoryPoint()
        jtp = copy.deepcopy(theplan.joint_trajectory.points[ii])
        jtp.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
        jtp.accelerations =[0.0, 0.0, 0.0, 0.0, 0.0]
        jtp.time_from_start = rospy.Duration.from_sec(secs + dt*(ii+1))
        jt.points.append(jtp)
    return jt
  
def make_trajectory_goal_msg(joint_trajectory_plan=[], seq=0, secs=0, nsecs=0, dt=2, frame_id='/base_link'):
    ''' 
    Converts a joint trajectory plan to a FollowJoinTrajectoryActionGoal message
    which is compatible with the topic /arm_N/arm_controller/follow_joint_trajectory/goal
    '''
    # Goal ID Generator
    id_gen = actionlib.GoalIDGenerator()
    
    # create joint trajectory message
    jtg = FollowJointTrajectoryActionGoal(goal_id=id_gen.generate_ID())
    
    # fill the header
    jtg.header.seq = seq
    jtg.header.stamp.secs = 0 #secs
    jtg.header.stamp.nsecs = 0 #nsecs
    jtg.header.frame_id = frame_id
    
    # specify the joint names
    jtg.goal.trajectory.joint_names = joint_trajectory_plan.joint_trajectory.joint_names
    
    # fill the trajectory points and computer constraint times
    njtp = len(joint_trajectory_plan.joint_trajectory.points)
    for ii in range(0, njtp):
        jtp = JointTrajectoryPoint()
        jtp = copy.deepcopy(joint_trajectory_plan.joint_trajectory.points[ii])
        jtp.time_from_start = rospy.Duration().from_sec(secs + dt*(ii+1))
        jtg.goal.trajectory.points.append(jtp)
        
    '''jtg.goal.goal_tolerance = JointTolerance()
    jtg.goal.goal_tolerance.name = "goaltol"
    jtg.goal.goal_tolerance.position = 0.02
    jtg.goal.goal_tolerance.velocity = 0.5
    jtg.goal.goal_tolerance.acceleration = 0.5
    
    jtg.goal.path_tolerance = JointTolerance()
    jtg.goal.path_tolerance.name = "goaltol"
    jtg.goal.path_tolerance.position = 0.02
    jtg.goal.path_tolerance.velocity = 0.5
    jtg.goal.path_tolerance.acceleration = 0.5
        
    jtg.goal.goal_time_tolerance = rospy.Duration()
    jtg.goal.goal_time_tolerance.set(5, 0)'''
    return jtg  
  
def get_random_waypoints(group, eef_link, n):  
  '''
  Builds a list of random waypoints
  '''
  poses = []
  #poses.append(copy.deepcopy(group.get_current_pose().pose))
  for ii in range(0,n):
    pose = group.get_random_pose(eef_link)
    poses.append(copy.deepcopy(pose.pose))
  return poses

def JointTrajectory_publisher(topic_name, msg_type):
  '''
  Main MoveIt! publisher demo function
  '''
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('moveit_client',
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
 
  ## Create a publisher to gazebo arm controller
  rospy.loginfo("Publishing to " + topic_name)
  gazebo_command_publisher = rospy.Publisher(
                                      topic_name,
                                      msg_type,
                                      queue_size=10 ) 
    
  # setup the loop rate for the node
  r = rospy.Rate(10) # 10hz
  
  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  #rospy.sleep(10)
  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Planning reference frame: %s" % group.get_planning_frame()
  
  ## We can also print the name of the end-effector link for this group
  print "============ End effector link: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"
  
  '''
  This section builds a set of waypoints.  The joint trajectory (the plan) will be
  computed using these waypoints.
  '''
  # POSE 1
  end_effector_link = "gripper_palm_link"
  theta = 0 #math.pi
  psi = 0 #math.pi/4
  quat = quaternion_from_euler(0,theta,psi)
  P = (0.0,0.0,0.35)
  pose1 = geometry_msgs.msg.PoseStamped()
  pose1.header.frame_id = "base_link"
  pose1.pose.position.x = P[0]
  pose1.pose.position.y = P[1]
  pose1.pose.position.z = P[2]
  pose1.pose.orientation.x = quat[0]
  pose1.pose.orientation.y = quat[1]
  pose1.pose.orientation.z = quat[2]
  pose1.pose.orientation.w = quat[3]  
  print "Pose 1 is: "
  print pose1  
    
    
  # POSE 2
  theta = -math.pi
  psi = math.pi/4
  quat = quaternion_from_euler(0,theta,psi)
  P = (0.2,0.2,0.07)
  pose2 = geometry_msgs.msg.PoseStamped()
  pose2.header.frame_id = "base_link"
  pose2.pose.position.x = P[0]
  pose2.pose.position.y = P[1]
  pose2.pose.position.z = P[2]
  pose2.pose.orientation.x = quat[0]
  pose2.pose.orientation.y = quat[1]
  pose2.pose.orientation.z = quat[2]
  pose2.pose.orientation.w = quat[3]
  print "Pose 2 is: "
  print pose2 
  
  # Pose list
  poses = []
  poses.append(copy.deepcopy(pose1.pose))
  poses.append(copy.deepcopy(pose2.pose))
  
  # Enforce that the poses are specified in reference to the base_link
  group.set_pose_reference_frame("base_link")
  
  # Uncomment if all waypoints are to be used.
  # Note that the default planner seems to disregard 
  #  all points except the last one
  group.set_pose_targets(poses, end_effector_link)
  
  # Uncomment if one specific pose will be used
  #group.set_pose_target(pose1, end_effector_link)
  
  # Uncomment this block to loop until a valid path is found
  #  This is for demonstration purposes only
  #  All previous path specifications are ignored
  '''
  is_good = False
  while not is_good:
    end_effector_link = "gripper_palm_link"
    pose = group.get_random_pose(end_effector_link)
    #pose = one_good_pose()
    group.set_pose_target(pose.pose)
    print "The pose is: "
    print pose
    plan = group.plan()
    if len(plan.joint_trajectory.points) > 0:
      is_good = True
      rospy.loginfo("Found valid trajectory")
  '''
  
  # Call Moveit! planner to produce a trajectory
  plan = group.plan()
  if len(plan.joint_trajectory.points) == 0:
    return None  # We exist if a plan is not found
    
  #(plan, fraction) = group.compute_cartesian_path(pose, eef_step=0.01, jump_threshold=0.0, avoid_collisions=True)
  #print "========= Here be the plan\n"
  #print plan

  #jt = make_trajectory_msg(plan=plan, secs=1, dt=0.2, frame_id='base_link' )
  jt = make_trajectory_goal_msg(joint_trajectory_plan=plan, secs=1, dt=0.1, frame_id='base_link' )
  
  rospy.loginfo("Publishing joint trajectory")
  gazebo_command_publisher.publish(jt)
  r.sleep()
  
  # Note that sometimes, goal execution will not work without publishing more than once.
  # This is very strange since the topics are setup to be implemented for use in 
  #  an action server.
  # TODO: Implement an action server to handle execution of joint trajectories.
  #  Might want to implement as a service instead of an action server, but action server
  #  should be more interesting for security testing.
  while not rospy.is_shutdown():
    gazebo_command_publisher.publish(jt)
    r.sleep()  

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
    JointTrajectory_publisher(
         topic_name="/arm_1/arm_controller/follow_joint_trajectory/goal", 
         msg_type=FollowJointTrajectoryActionGoal)    
    
  except rospy.ROSInterruptException:
    pass


