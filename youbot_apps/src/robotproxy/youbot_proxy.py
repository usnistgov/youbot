
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
from brics_actuator.msg import JointPositions, JointValue
from sensor_msgs.msg import JointState
import numpy

class YoubotProxy(BaseProxy):
    
    # class attributes
    arm_joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
    arm_joint_indexes = [0,1,2,3,4]
    gripper_joint_names = ['gripper_finger_joint_l', 'gripper_finger_joint_r']
    gripper_joint_indexes = [5,6]
    #TODO delete: end_effector_link = "gripper_pointer_link"    
    
    def __init__(self, node_name):
        rospy.logdebug("YoubotProxy __init__")
        super(YoubotProxy,self).__init__()
        self.init_done = False  # indicates that the object was initialized 
        
        # init ros node
        rospy.init_node(node_name, anonymous=True)
        rospy.loginfo("ROS node initialized: " + rospy.get_name())
        rospy.loginfo("node namespace is : " + rospy.get_namespace())
        rospy.loginfo("node uri is : " + rospy.get_node_uri())
        
        # init object attributes
        self.arm_num = rospy.get_param("~arm_num")
        self.gripper_move_duration = rospy.get_param("~gripper_move_duration",10)
        self.arm_move_duration = rospy.get_param("~arm_move_duration",10)

        # init joint_states subscriber
        self._joint_states_arm = [0]*5
        self._joint_states_gripper = [0]*2        
        # todo: create thread event object for joint_states variable
        self._joint_states_topic = rospy.get_param("~joint_states_topic")
        self._joint_states_sub = rospy.Subscriber(self._joint_states_topic, JointState, self.joint_states_cb)  

        # Gripper distance tolerance: 1 mm 
        self.gripper_distance_tol = rospy.get_param("~gripper_distance_tol", 0.001) 
        # Joint distance tolerance: 1/10 radian tolerance (1.2 degrees)
        self.arm_joint_distance_tol = rospy.get_param("~joint_distance_tol",0.025)
        
        # init moveit
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
            self.arm_group.set_planning_time(8)
            self.arm_group.set_pose_reference_frame("base_link")
            rospy.loginfo("planning group created for manipulator")
        except:
            pass

        # init arm publisher
        self._arm_pub = rospy.Publisher("arm_" + str(self.arm_num) + "/arm_controller/position_command", JointPositions)
        rospy.loginfo("Created arm joint position publisher ")
        
        # init gripper action client
        self._gripper_pub = rospy.Publisher("arm_" + str(self.arm_num) + "/gripper_controller/position_command", JointPositions)
        rospy.loginfo("Created gripper joint position publisher ")
        
        # set init done flag
        self.init_done = True
                
    def joint_states_cb(self, data):
        try:
            # todo: wait for lock release
            self._joint_states_arm = [ data.position[i] for i in self.arm_joint_indexes ]
            self._joint_states_gripper = [ data.position[i] for i in self.gripper_joint_indexes ]
        except Exception as e:
            raise e
        finally:
            # add threading event in BaseProxy to lock the variable until operating completes
            #   use finaly to ensure lock is released
            pass
                            
    def plan_arm(self, pose): 
        '''
        @param pose: a PoseStamped object
        @precondition: initialize_node must be called first 
        @return: boolean
        @note: sets the intended goal in self
        '''
        raise NotImplementedError 

    @classmethod
    def make_brics_msg_arm(cls, positions):
        # create joint positions message
        jp = JointPositions()
        for ii in range(5):
            jv = JointValue()
            jv.joint_uri = 'arm_joint_' + str(ii+1)
            jv.unit='rad' 
            jv.value = positions[ii]
            jp.positions.append(copy.deepcopy(jv))
        return jp

    def move_arm(self, positions):    
        self._arm_goal = copy.deepcopy(positions)
        jp = YoubotProxy.make_brics_msg_arm(positions)
        rospy.logdebug("brics message created")
        rospy.logdebug(jp)
        r = rospy.Rate(50)
        t0 = rospy.Time().now()
        while not rospy.is_shutdown():
            d = BaseProxy.measure_joint_distance_sum(self._joint_states_arm, positions)
            if (d < self.arm_joint_distance_tol):
                rospy.loginfo("moved arm to joint position error of: " + str(d))
                break        
            td = rospy.Time().now()
            if (td-t0 > self.arm_move_duration):
                rospy.loginfo("move arm timeout.  joint position error " + str(d))
                break        
            self._arm_pub.publish(jp)
            r.sleep()

    @classmethod
    def make_brics_msg_gripper(cls, opening_m):
        
        # Turn a desired gripper opening into a brics-friendly message    
        left = opening_m/2.0
        right = opening_m/2.0
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

    def move_gripper(self, opening_mm): 
        # ALERT! Make sure width of opening is no larger than 0.023 m (23 mm)
        if opening_mm > 23:
            raise Exception("gripper opening is too large: " + str(opening_mm))
        opening_m = opening_mm/1000.0
        jp = YoubotProxy.make_brics_msg_gripper(opening_m)
        rospy.logdebug("make_brics_msg_gripper position message")
        rospy.logdebug(jp)

        # gripper publisher
        r = rospy.Rate(30) 
        rospy.loginfo("moving gripper")
        t0 = rospy.Time().now()
        while not rospy.is_shutdown():
            self._gripper_pub.publish(jp)
            d = self.measure_gripper_distance(opening_m)
            #rospy.loginfo("moved gripper to position error of: " + str(d))
            if d < self.gripper_distance_tol:
                rospy.loginfo("moved gripper to position error of: " + str(d))
                break
            td = rospy.Time().now()
            if (td-t0 > self.gripper_move_duration):
                rospy.loginfo("move gripper timeout.  gripper position error " + str(d))
                break                
            r.sleep()

    def measure_gripper_distance(self, opening_m):
        curr_opening_m = float(self._joint_states_gripper[0]) + float(self._joint_states_gripper[1])
        d = abs(opening_m - curr_opening_m)
        return d 

    def control_loop(self):
        if self.commands is None:
            raise Exception('Command list is empty.  Was the control plan loaded?')
        
        # loop through the command list
        for cmd in self.commands:

            # wait for proxy to be in active state
            self.wait_for_state(self._proxy_state_running)
            
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
                       
            # check for any wait depends
            self.wait_for_depend(cmd)

            # execute command
            # could do this with a dictionary-based function lookup, but who cares
            if t == 'noop':
                rospy.loginfo("Command type: noop")
                self.wait_for_depend(cmd)
            elif t == 'sleep':
                rospy.loginfo("sleep command")
                v = float(spec)
                rospy.sleep(v)
            elif t == 'move_gripper':
                rospy.loginfo("gripper command")
                self.move_gripper(spec)
            elif t == 'move_arm':
                rospy.loginfo("move_arm command")
                rospy.logdebug(spec)
                self.move_arm(spec)
            elif t == 'plan_exec_arm':
                rospy.loginfo("plan and execute command not implemented")
                raise NotImplementedError()
            else:
                raise Exception("Invalid command type: " + str(cmd.type))

            # check for any set dependencies action
            self.set_depend(cmd)

            # check for any clear dependencies action
            self.clear_depend(cmd)





