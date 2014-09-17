
'''
@author: Rick Candell
@contact: rick.candell@nist.gov
@organization: NIST
@license: public domain
'''

import sys
import copy
import rospy
import moveit_commander
from base_proxy import BaseProxy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib

class YoubotGazeboProxy(BaseProxy):
    
    # class attributes
    arm_joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
    gripper_joint_names = ['gripper_finger_joint_l', 'gripper_finger_joint_r']
    end_effector_link = "gripper_pointer_link"    
    
    def __init__(self):
        rospy.logdebug("YoubotGazeboProxy __init__")
        super(YoubotGazeboProxy,self).__init__()
        self.init_done = False  # indicates that the object was initialized 
    
    def initialize_node(self, node_name, arm_id_num):
        
        # init object attributes
        self._arm_as_name = '/arm_' +str(arm_id_num) + '/arm_controller/follow_joint_trajectory'
        self._gripper_as_name = '/arm_' +str(arm_id_num) + '/gripper_controller/follow_joint_trajectory'        
        
        # init moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
        self.arm_group.set_planning_time(8)
        self.arm_group.set_pose_reference_frame("base_link")
        rospy.loginfo("planning group created for manipulator")
        
        #init ros node
        rospy.init_node(node_name, anonymous=False)
        rospy.loginfo("ROS node initialized: " + rospy.get_name())
        rospy.loginfo("node namespace is : " + rospy.get_namespace())
        
        # init arm action client
        self._ac_arm = actionlib.SimpleActionClient(self._arm_as_name, FollowJointTrajectoryAction)
        self._ac_arm.wait_for_server()
        rospy.loginfo("Created arm joint trajectory action client " + self._arm_as_name)
        
        # init gripper action client
        self._ac_gripper = actionlib.SimpleActionClient(self._gripper_as_name, FollowJointTrajectoryAction)
        self._ac_gripper.wait_for_server()
        rospy.loginfo("Created gripper joint trajectory action client " + self._gripper_as_name)
        
        # set init done flag
        self.init_done = True
                
                
    def plan_arm(self, pose): 
        '''
        @param pose: a PoseStamped object
        @precondition: initialize_node must be called first 
        @return: boolean
        @note: sets the intended goal in self
        '''
        # test for initialization
        if self.init_done == False:
            raise Exception("Object not initialized")
        
        # perform planning action 
        self.arm_group.clear_pose_targets()
        self.arm_group.set_pose_target(pose, self._end_effector_link)
        plan = self.arm_group.plan()
        
        if len(plan.joint_trajectory.points) == 0:
            rospy.loginfo("plan not found for given pose")
            self._arm_goal = None
            return False
        
        else:
            rospy.loginfo("number of points in trajectory: " + str(len(plan.joint_trajectory.points)))
            self._arm_goal = FollowJointTrajectoryGoal()
            self._arm_goal.trajectory = copy.deepcopy(plan.joint_trajectory)
            return True        
                    
    def move_arm(self):
        # Sends the goal to the action server.
        self._ac_arm.send_goal(self._arm_goal, feedback_cb=self.move_arm_feedback_cb)
    
        # Waits for the server to finish performing the action.
        self._ac_arm.wait_for_result()  
    
        # get the result code
        return self._ac_arm.get_result()    
    
    def move_arm_feedback_cb(self): pass

    def move_gripper(self, opening_mm): 
        # Creates a goal to send to the action server.
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self._gripper_joint_names 
        jtp = JointTrajectoryPoint()
        jtp.positions = [opening_mm/2000, opening_mm/2000]
        jtp.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(jtp)
        self._ac_gripper.send_goal(goal)
        self._ac_gripper.wait_for_result()
        return self._ac_gripper.get_result()
        
    def execute_control(self):
        if self.commands is None:
            raise Exception('Command list is empty.  Was the control plan loaded?')
        
        # loop through the command list
        for cmd in self.commands:
            t = cmd['type']
            spec = self.positions[cmd['spec']]
            if t == 'sleep':
                v = float(spec)
                self.sleep(v)
            elif t == 'move_gripper':
                self.move_gripper(spec/1000.0)
            elif t == 'move_arm':
                rospy.logdebug("Received joint spec: ")
                rospy.logdebug(cmd.spec)                
                goal = FollowJointTrajectoryGoal()
                goal.trajectory.joint_names = self.arm_joint_names
                jtp = JointTrajectoryPoint()
                jtp.positions = spec
                goal.trajectory.points.append(jtp)
                self._arm_goal = copy.deepcopy(goal)
                self.move_arm()
            elif t == 'plan_exec_arm':
                raise NotImplementedError()
                '''
                pose = PoseStamped()
                q = quaternion_from_euler(cmd.spec[3],cmd.spec[4],cmd.spec[5])
                pose.header.frame_id = "base_link"
                pose.pose.position.x = cmd.spec[0]
                pose.pose.position.y = cmd.spec[1]
                pose.pose.position.z = cmd.spec[2]
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]  
                if self.plan_arm(pose):
                    rv = self.move_arm()
                    rospy.logdebug("Move arm returned code " + str(rv))
                else:
                    rospy.logerr("trajectory not found")
                    rospy.logdebug("pose specified to planner: ")
                    rospy.logdebug(pose)
                    raise Exception("trajectory not found")
                    '''
            else:
                raise Exception("Invalid command type: " + str(cmd.type))





