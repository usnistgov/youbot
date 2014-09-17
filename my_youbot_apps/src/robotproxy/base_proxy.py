
'''
@author: Rick Candell
@contact: rick.candell@nist.gov
@organization: NIST
@license: public domain
'''
    
from abc import ABCMeta, abstractmethod
import copy
import rospy
from joint_pose_dict import JointPoseDictionary
from command_sequence import CommandSequence

class BaseProxy(object):
    
    __metaclass__ = ABCMeta
    
    def __init__(self):
        self.positions = None   # stores the dictionary of joint positions
        self.commands =  None   # stores the list of commands to execute
        self._arm_joint_names = None     # names of joints in arm
        self._gripper_joint_names = None     # names of joints in gripper
        self._arm_goal = None       # contains the current goal for the arm 
        self._gripper_goal = None   # contains the current goal for the gripper
    
    def load_control_plan(self, path_to_dict_yaml, path_to_cmds_yaml):
        # load the joint positions dictionary 
        self.positions = JointPoseDictionary(path_to_dict_yaml)
        print self.positions
        self.commands = CommandSequence(path_to_cmds_yaml)
        print self.commands
    
    def sleep(self, rate_obj, fsecs):
        rospy.sleep(fsecs)

    @property
    def arm_joint_names(self): 
        return self._arm_joint_names
    @arm_joint_names.setter
    def arm_joint_names(self, names):
        self._arm_joint_names = copy.deepcopy(names)
        
    @property
    def gripper_joint_names(self): 
        return self._gripper_joint_names
    @gripper_joint_names.setter
    def gripper_joint_names(self, names):
        self._gripper_joint_names = copy.deepcopy(names)    
    
    @abstractmethod
    def initialize_node(self, node_name, arm_id_num): raise NotImplementedError()
    
    @abstractmethod
    def plan_arm(self, pose): raise NotImplementedError()
    # pose is a geometry_msg.Pose object
    # group is one of the planning groups stored in self 
        
    @abstractmethod
    def move_arm(self): raise NotImplementedError()
    # trajectory is a JointTrajectory object

    @abstractmethod
    def move_gripper(self, opening_mm): raise NotImplementedError()
    # opening_mm is a float indicating distance between gripper links
        
    @abstractmethod
    def execute_control(self): raise NotImplementedError()
    
    
    
    
    