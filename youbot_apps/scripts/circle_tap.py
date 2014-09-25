#!/usr/bin/env python

'''
 @author: Rick Candell
 @contact: rick.candell@nist.gov
 @organization: NIST
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

def massage_traj(traj_in):
    traj_out = JointTrajectory()
    traj_out.joint_names = traj_in.joint_names
    npts = len(traj_in.points)

    # take first point
    traj_out.points.append(copy.deepcopy(traj_in.points[0]))
    #traj_out.points[0].velocities = [0,0,0,0,0]
    #traj_out.points[0].accelerations = [0,0,0,0,0]

    # take middle point
    #  midpt = int(npts/2)
    #  traj_out.points.append(copy.deepcopy(traj_in.points[midpt]))
    #  traj_out.points[1].velocities = [0,0,0,0,0]
    #  traj_out.points[1].accelerations = [0,0,0,0,0]

    # take last point
    traj_out.points.append(copy.deepcopy(traj_in.points[-1]))
    #traj_out.points[-1].velocities = [0,0,0,0,0]
    #traj_out.points[-1].accelerations = [0,0,0,0,0]

    rospy.loginfo("Points in traj: " + str(len(traj_out.points)))
    return traj_out


def make_brics_msg_gripper(opening_m):
    # Turn a desired gripper opening into a brics-friendly message    
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
    jp = make_brics_msg_gripper(opening_m)
    print jp
    # Initialize the timer for gripper publisher
    t0 = rospy.get_rostime  # TODO: This looks wrong - rc
    while not rospy.is_shutdown():
        rospy.loginfo("moving gripper")
        gripper_command_publisher.publish(jp) 
        #r.sleep()
        t1 = rospy.get_rostime
        if t1 > 8:
            break

def plan_arm(client, group, pose, end_effector_link):

    result_code = None
    group.clear_pose_targets()
    group.set_pose_target(pose, end_effector_link)
    #Sgroup.set_goal_position_tolerance(0.01)
    #group.set_goal_orientation_tolerance(0.05)     
    plan = group.plan()

    if len(plan.joint_trajectory.points) == 0:
        rospy.loginfo("Trajectory not found for given pose and solver")
        return None

    else:
        rospy.loginfo("number of points in traj: " + str(len(plan.joint_trajectory.points)))
        plan.joint_trajectory = trajectory_toffset(plan.joint_trajectory, 0)
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory = copy.deepcopy(plan.joint_trajectory)
        goal.trajectory = massage_traj(goal.trajectory)
        return goal
    
def move_arm_as(client, goal):
    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=jtg_feedback_cb)

    # Waits for the server to finish performing the action.
    client.wait_for_result()  

    # get the result code
    return client.get_result()     
    
def make_brics_msg_arm(positions):
    # create joint positions message
    jp = JointPositions()
    for ii in range(5):
        jv = JointValue()
        jv.joint_uri = 'arm_joint_' + str(ii+1)
        jv.unit='rad' 
        jv.value = positions[ii]
        jp.positions.append(copy.deepcopy(jv))
    return jp
    
def move_arm_brics(arm_pub, goal, dur=3.0):    
    # get the positions from the last point
    positions = goal.trajectory.points[-1].positions
    jp = make_brics_msg_arm(positions)
    rospy.logdebug("brics message created")
    rospy.logdebug(jp)
    r = rospy.Rate(50)
    t0 = rospy.get_rostime()
    while not rospy.is_shutdown():
        if rospy.get_rostime().to_sec() - t0.to_sec() < dur:
            arm_pub.publish(jp)
            r.sleep()
        else:
            break

def JointTrajectory_client(USE_BRICS=False, 
                           arm_action_server_name='/arm_1/arm_controller/follow_joint_trajectory', 
                           gripper_action_server_name='/arm_1/gripper_controller/follow_joint_trajectory'):

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
    group.set_planning_time(8)
    #print "Current pose"
    #print group.get_current_pose()
    #print "Current rpy"
    #print group.get_current_rpy()
    #return None

    ## Create a client for the action server
    rospy.loginfo("Create arm joint trajectory action client " + arm_action_server_name)
    arm_client = actionlib.SimpleActionClient(arm_action_server_name, 
                    control_msgs.msg.FollowJointTrajectoryAction)

    if USE_BRICS == True:
        # create an arm brics_actuator publisher
        arm_pub = rospy.Publisher("/arm_1/arm_controller/position_command", JointPositions, queue_size = 5)
        rospy.loginfo("Created publisher for /arm_1/arm_controller/position_command")
        arm_pub_dur = 3.0

    else:
    
        #gripper_client = actionlib.SimpleActionClient(gripper_action_server_name, 
        #                                      control_msgs.msg.FollowJointTrajectoryAction)
        #rospy.loginfo("Created gripper joint trajectory action client " + gripper_action_server_name)

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

    # Enforce that the poses are specified in reference to the base_link
    group.set_pose_reference_frame("base_link")
    end_effector_link = "gripper_pointer_link"

    # set protocol flag 
    if USE_BRICS:
        rospy.loginfo("using BRICS protocol")
    else:
        rospy.loginfo("using AS protocol")
    
    # move to the ready pose
    ready_goal = control_msgs.msg.FollowJointTrajectoryGoal()
    ready_goal.trajectory.joint_names = ['arm_joint_1','arm_joint_2','arm_joint_3','arm_joint_4','arm_joint_5']
    ready_jtp = JointTrajectoryPoint()
    ready_jtp.positions = [2.95,0.7,-2.0,2.1,0.15]
    ready_jtp.time_from_start = rospy.Duration(0.25)
    ready_goal.trajectory.points.append(ready_jtp)
    if USE_BRICS:
        move_arm_brics(arm_pub, ready_goal, arm_pub_dur)
    else:
        move_arm_as(arm_client, ready_goal)

    # create a pose object with default settings
    roll = 0
    pitch = -math.pi
    yaw = math.pi/4
    quat = quaternion_from_euler(roll,pitch,yaw)    
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]        
    
    # create the position vectors for the poses
    from numpy import arange, cos, sin
    rad = 0.2 # meters
    zup = 0.06
    zdn = 0.03    
    npts = 10
    # angles in first and second quadrants only
    theta_vec = -(math.pi*arange(npts+1)/float(npts)-math.pi/2.0)
    xvec = rad*cos(theta_vec)
    yvec = rad*sin(theta_vec)
    up_status= [False]*npts
    up_goal = [False]*npts
    dn_status = [False]*npts    
    dn_goal = [False]*npts    
    
    while True:

        for ii in range(npts):
            
            if rospy.is_shutdown():
                return (-1)

            # compose the pose object    
            pose.pose.position.x = xvec[ii]
            pose.pose.position.y = yvec[ii]
            pose.pose.position.z = zup
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]    

            rospy.loginfo("Up status: " + str(up_status))
            if up_status[ii] is False:
                # work the up pose
                rospy.loginfo("computing plan for up pose")
                rospy.logdebug(pose)                
                goal = plan_arm(arm_client, group, pose, end_effector_link)
                if type(goal) is control_msgs.msg.FollowJointTrajectoryGoal:
                    rospy.loginfo("UP plan found")
                    rospy.logdebug(goal)                
                    up_status[ii] = True
                    up_goal[ii] = goal
                    if USE_BRICS:
                        move_arm_brics(arm_pub, up_goal[ii], arm_pub_dur)
                    else:
                        move_arm_as(arm_client, up_goal[ii])
                    
            else:
                rospy.loginfo("using stored up goal")
                if USE_BRICS:
                    move_arm_brics(arm_pub, up_goal[ii], arm_pub_dur)
                else:
                    move_arm_as(arm_client, up_goal[ii])
                
                
            if up_status[ii] is not False:
                # work the down pose
                if dn_status[ii] is False:
                    pose.pose.position.z = zdn
                    rospy.loginfo("computing plan for down pose")
                    rospy.logdebug(pose)                            
                    goal = plan_arm(arm_client, group, pose, end_effector_link)            
                    if type(goal) is control_msgs.msg.FollowJointTrajectoryGoal:
                        rospy.loginfo("DOWN plan found")
                        rospy.logdebug(goal)    
                        dn_status[ii] = True
                        dn_goal[ii] = goal 
                        if USE_BRICS:
                            move_arm_brics(arm_pub, dn_goal[ii], arm_pub_dur)
                            move_arm_brics(arm_pub, up_goal[ii], arm_pub_dur)
                        else:
                            move_arm_as(arm_client, dn_goal[ii])
                            
                else:
                    rospy.loginfo("using stored down goal")
                    if USE_BRICS:
                        move_arm_brics(arm_pub, dn_goal[ii], arm_pub_dur)
                        move_arm_brics(arm_pub, up_goal[ii], arm_pub_dur)
                    else:
                        move_arm_as(arm_client, dn_goal[ii])
                
    return None

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

    # Exit the MoveIt commander gracefully.
    #  This is a bug work-around.  It's supposed to prevent a core dump upon exiting.
    moveit_commander.os._exit(0)

# 
# This is the main subroutine
#  
if __name__=='__main__':
    use_brics = False
    if sys.argv[1]=="true":
        use_brics = True
    try:
        #JointTrajectory_publisher(
        #     topic_name="/arm_1/arm_controller/command", 
        #     msg_type=JointTrajectory)     
        JointTrajectory_client(use_brics, '/arm_1/arm_controller/follow_joint_trajectory',
                   '/arm_1/gripper_controller/follow_joint_trajectory')

    except rospy.ROSInterruptException:
        moveit_commander.os._exit(0)



