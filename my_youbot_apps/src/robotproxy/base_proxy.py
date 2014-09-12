
'''
@author: Rick Candell
@contact: rick.candell@nist.gov
@organization: NIST
@license: public domain
'''

from abc import ABCMeta, abstractmethod

class BaseProxy(object):
    
    __metaclass__ = ABCMeta

    @property
    def _arm_joint_names(self): raise NotImplementedError
    @property
    def _gripper_joint_names(self): raise NotImplementedError
        
    @abstractmethod
    def __init__(self): pass
    
    @abstractmethod
    def initialize_node(self, node_name, arm_id_num): pass
    
    @abstractmethod
    def plan_arm(self, group, pose): pass
    # pose is a geometry_msg.Pose object
    # group is one of the planning groups stored in self 
        
    @abstractmethod
    def move_arm(self, trajectory): pass
    # trajectory is a JointTrajectory object

    @abstractmethod
    def move_gripper(self, opening_mm): pass
    # opening_mm is a float indicating distance between gripper links
            
    @abstractmethod
    def load_control_plan(self, path): pass
        
    @abstractmethod
    def execute_control(self): pass
    
    