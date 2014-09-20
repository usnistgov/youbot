
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
from base_proxy import BaseProxy, ProxyCommand
#from geometry_msgs.msg import PoseStamped
#from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib

class YoubotGazeboProxy(BaseProxy):
    
    # class attributes
    arm_joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
    gripper_joint_names = ['gripper_finger_joint_l', 'gripper_finger_joint_r']
    end_effector_link = "gripper_pointer_link"    
    
    def __init__(self, node_name, arm_id_num):
        rospy.logdebug("YoubotGazeboProxy __init__")
        super(YoubotGazeboProxy,self).__init__(arm_id_num)
        self.init_done = False  # indicates that the object was initialized 
    
        # init object attributes
        self._arm_as_name = '/arm_' +str(arm_id_num) + '/arm_controller/follow_joint_trajectory'
        self._gripper_as_name = '/arm_' +str(arm_id_num) + '/gripper_controller/follow_joint_trajectory'        
        
        # init moveit
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
            self.arm_group.set_planning_time(8)
            self.arm_group.set_pose_reference_frame("base_link")
            rospy.loginfo("planning group created for manipulator")
        except:
            pass
        
        #init ros node
        rospy.init_node(node_name, anonymous=False)
        rospy.loginfo("ROS node initialized: " + rospy.get_name())
        rospy.loginfo("node namespace is : " + rospy.get_namespace())
        rospy.loginfo("node uri is : " + rospy.get_node_uri())
        
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
        goal.trajectory.joint_names = self.gripper_joint_names
        jtp = JointTrajectoryPoint()
        jtp.positions = [opening_mm/2000.0, opening_mm/2000.0]
        jtp.velocities = [0, 0]
        jtp.time_from_start = rospy.Duration(1.0)
        goal.trajectory.points.append(jtp)
        self._ac_gripper.send_goal(goal)
        self._ac_gripper.wait_for_result()
        return self._ac_gripper.get_result()
              
    def control_loop(self):
        if self.commands is None:
            raise Exception('Command list is empty.  Was the control plan loaded?')
        
        # loop through the command list
        for cmd in self.commands:
            
            # process commands
            cmd_spec_str = None
            spec = None
            t = cmd[ProxyCommand.key_command_type]
            if not (t == "noop"):
                cmd_spec_str = cmd[ProxyCommand.key_command_spec]
                if not isinstance(cmd_spec_str, basestring):
                    spec = float(cmd_spec_str)
                else:
                    spec = self.positions[cmd_spec_str]
                rospy.loginfo("Command type: " + t + ", spec: " + str(cmd_spec_str) + ", value: " + str(spec))
                       
            # execute command
            # could do this with a dictionary-based function lookup, but who cares
            if t == 'noop':
                rospy.loginfo("Command type: noop")
                self.wait_for_depend(cmd)
                self.set_depend(cmd, True)            
            elif t == 'sleep':
                rospy.loginfo("sleep command")
                self.wait_for_depend(cmd)
                v = float(spec)
                rospy.sleep(v)
                self.set_depend(cmd, True)
            elif t == 'move_gripper':
                rospy.loginfo("gripper command")
                self.wait_for_depend(cmd)
                self.move_gripper(spec)
                self.set_depend(cmd, True)
            elif t == 'move_arm':
                rospy.loginfo("move_arm command")
                rospy.logdebug(spec)                
                goal = FollowJointTrajectoryGoal()
                goal.trajectory.joint_names = self.arm_joint_names
                jtp = JointTrajectoryPoint()
                jtp.time_from_start = rospy.Duration(0.5)  # fudge factor for gazebo controller
                jtp.positions = spec
                jtp.velocities = [0]*len(spec)
                goal.trajectory.points.append(jtp)
                self._arm_goal = copy.deepcopy(goal)
                self.move_arm()
                self.set_depend(cmd, True)
            elif t == 'plan_exec_arm':
                rospy.loginfo("plan and execute command not implemented")
                raise NotImplementedError()
            else:
                raise Exception("Invalid command type: " + str(cmd.type))





