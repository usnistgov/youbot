
'''
@author: Rick Candell
@contact: rick.candell@nist.gov
@organization: NIST
@license: public domain
'''
    
from abc import ABCMeta, abstractmethod
import copy
import rospy
import numpy
from joint_pose_dict import JointPoseDictionary
from command_sequence import CommandSequence
from proxy_depend import ProxyDepends

class ProxyCommand():
    key_command_type = "type"
    key_command_spec = "spec"
    key_command_wait_depend = "wait_depend"
    key_command_set_depend = "set_depend"

    SYS_BEGIN = "/system/begin"

class BaseProxy(object):
    
    __metaclass__ = ABCMeta

    _proxy_state_stopped = -1
    _proxy_state_running = 1
    
    def __init__(self):
        '''
        @todo: implement emergency stop
        '''
        self.proxy_state = self._proxy_state_running
        self.arm_num = None
        self._frame_id = "base_link"
        self.positions = None   # stores the dictionary of joint positions
        self.commands =  None   # stores the list of commands to execute
        self._arm_joint_names = None     # names of joints in arm
        self._gripper_joint_names = None     # names of joints in gripper
        self._end_effector_link = None   # name of the end effector link
        self._frame_id = None       # name of the base frame
        self._arm_goal = None       # contains the current goal for the arm 
        self._gripper_goal = None   # contains the current goal for the gripper
        self.depends_status = ProxyDepends(self.arm_num) 

        # this will be a dictionary of Thread Event objects
        self.locks = {}

    def reset_depend_status(self):
        rospy.loginfo("call to reset dependency database in base_proxy")
        self.depends_status.reset_database()
        
    def wait_for_state(self, state):
        while True: 
            if self.proxy_state == state: break

    def load_control_plan(self, path_to_dict_yaml, path_to_cmds_yaml):
        # load the joint positions dictionary 
        self.positions = JointPoseDictionary(path_to_dict_yaml)
        rospy.logdebug(self.positions)
        self.commands = CommandSequence(path_to_cmds_yaml)
        rospy.logdebug(self.commands)
    
    def sleep(self, rate_obj, fsecs):
        rospy.sleep(fsecs)

    def wait_for_system_begin(self):
        self.depends_status.wait_for_depend(ProxyCommand.SYS_BEGIN)
        
    def wait_for_depend(self, cmd):
        '''
        @param cmd: a dictionary containing type, spec, and optional depend 
        @todo: make this support a list of depends
        '''
        if cmd.has_key(ProxyCommand.key_command_wait_depend):
            wait_depend_name = cmd[ProxyCommand.key_command_wait_depend]
            self.depends_status.wait_for_depend(wait_depend_name)      

    def get_depend_status(self, name):
        return self.depends_status.get_depend_status(name)

    def clear_depend(self, cmd):
        '''
        @param cmd: a dictionary containing type, spec, and optional depend 
        @todo: make this support a list of depends
        '''
        if cmd.has_key(ProxyCommand.key_command_wait_depend):        
            self.depends_status.transmit_update_depend(cmd[ProxyCommand.key_command_wait_depend], False)

    def set_depend(self, cmd):
        '''
        @param cmd: a dictionary containing type, spec, and optional depend 
        @todo: make this support a list of depends
        '''
        if cmd.has_key(ProxyCommand.key_command_set_depend):        
            self.depends_status.transmit_update_depend(cmd[ProxyCommand.key_command_set_depend], True)

    @classmethod
    def measure_euclidean_distance(cls, x, y):
        '''
        @notes: measures euclidean distance of all joints combined
        '''
        # todo: add threading event to lock the variable until operating completes
        #   use finaly to ensure lock is released
        x = numpy.asfarray(x)
        y = numpy.asfarray(y)        
        d = numpy.sqrt(numpy.sum( (x-y)**2 ))
        return d

    @classmethod
    def measure_joint_distance_sum(cls, x, y):
        '''
        @notes: measures sum of all joint distances
        '''
        # todo: add threading event to lock the variable until operating completes
        #   use finaly to ensure lock is released
        x = numpy.asfarray(x)
        y = numpy.asfarray(y)     
        d = numpy.sum(numpy.abs(x-y))   
        return d        
    
    @classmethod
    def measure_joint_velocity_ss(cls, x):
        v = numpy.asfarray(x)
        d = numpy.sum(v**2)
        return d
            
    @property
    def frame_id(self): 
        return self._frame_id
    @frame_id.setter
    def frame_id(self, name):
        self._frame_id = name
        
    @property
    def end_effector_link(self): 
        return self._end_effector_link
    @end_effector_link.setter
    def end_effector_link(self, name):
        self._end_effector_link = name
        
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
    def control_loop(self): raise NotImplementedError()
             
    
    
    