
'''
@author: Rick Candell
@contact: rick.candell@nist.gov
@organization: NIST
@license: public domain
'''

import copy
import rospy
import moveit_commander
from base_proxy import *
from control_msgs.msg import *
from trajectory_msgs.msg import *
import actionlib
import yaml

class YoubotGazeboProxy(BaseProxy):
    
    # class attributes
    _arm_joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
    _gripper_joint_names = ['gripper_finger_joint_l', 'gripper_finger_joint_r']
    _end_effector_link = "gripper_pointer_link"    
    
    def __init__(self):
        rospy.logdebug("YoubotGazeboProxy __init__")
        self.positions = None   # stores the dictionary of joint positions
        self.commands = None    # stores the list of commands to execute
    
    def initialize_node(self, node_name, arm_id_num):
        
        # init object attributes
        self._arm_as_name = '/arm_' +str(arm_id_num) + '/arm_controller/follow_joint_trajectory'
        self._gripper_as_name = '/arm_' +str(arm_id_num) + '/gripper_controller/follow_joint_trajectory'        
        
        # init moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self._group = moveit_commander.MoveGroupCommander("manipulator")
        self._group.set_planning_time(5)
        self._group.set_pose_reference_frame("base_link")
        self._arm_goal = None
        self._gripper_goal = None
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
                
                
    def plan_arm(self, group, pose): 
        group.clear_pose_targets()
        group.set_pose_target(pose, self._end_effector_link)
        plan = group.plan()
        
        if len(self._arm_plan.joint_trajectory.points) == 0:
            rospy.loginfo("plan not found for given pose")
            self._arm_goal = None
            return False
        
        else:
            rospy.loginfo("number of points in trajectory: " + str(len(plan.joint_trajectory.points)))
            self._arm_goal = FollowJointTrajectoryGoal()
            self._arm_goal.trajectory = copy.deepcopy(plan.joint_trajectory)
            return True        
                    
    def move_arm(self, trajectory):
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
                    
    def load_control_plan(self, path_to_dict_yaml, path_to_cmds_yaml):
        # load the joint positions dictionary 
        f = open(path_to_dict_yaml)
        d = yaml.load(f)
        self.positions = copy.deepcopy(d)
        f.close()
        # load the command sequence for this robot
        f = open(path_to_dict_yaml)
        d = yaml.load(f)
        self.commands = copy.deepcopy(d)
        f.close()
        
    def execute_control(self):
        if self.commands is None:
            raise Exception('Command list is empty.  Was the control plan loaded?')
        
        # loop through the command list
        for cmd in self.commands:
            print type(cmd), cmd
        
       
            

        
        