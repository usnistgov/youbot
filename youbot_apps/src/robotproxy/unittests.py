
'''
@author: Rick Candell
@contact: rick.candell@nist.gov
@organization: NIST
@license: public domain
'''

import yaml
import robotproxy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

path_to_dict_yaml = '/home/rick/catkin_ws/src/youbot/youbot_apps/config/joint_pos_defs.yaml'
path_to_cmds_yaml = '/home/rick/catkin_ws/src/youbot/youbot_apps/config/arm1_commands.yaml'

def test_joint_pose_defs():
    f = open(path_to_dict_yaml)
    d = yaml.load(f)
    print type(d)   
    print d 
    for x in d.keys():
        print x, d[x] 

def test_commands_list():
    f = open(path_to_cmds_yaml)
    d = yaml.load(f)
    print type(d)   
    print d 
    for x in d:
        print type(x)
        print x

def test_load_control_plan():
    try:
        ygp = robotproxy.YoubotGazeboProxy("test_ygp", 1)
        ygp.load_control_plan(path_to_dict_yaml, path_to_cmds_yaml)
    except Exception as e:
        print e
    
def test_youbot_gazebo_proxy_move(): 
    ygp = robotproxy.YoubotGazeboProxy("test_ygp", 1)
    
    pose1 = PoseStamped()
    roll = 0
    pitch = 0 #math.pi
    yaw = 0 #math.pi/4
    quat = quaternion_from_euler(roll,pitch,yaw)
    P = (0.0,0.0,0.55)
    pose1.header.frame_id = "base_link"
    pose1.pose.position.x = P[0]
    pose1.pose.position.y = P[1]
    pose1.pose.position.z = P[2]
    pose1.pose.orientation.x = quat[0]
    pose1.pose.orientation.y = quat[1]
    pose1.pose.orientation.z = quat[2]
    pose1.pose.orientation.w = quat[3]  
    if ygp.plan_arm(pose1):
        rv = ygp.move_arm()
        print "return code is " + str(rv)
        print "test passed"
    else:
        print "test failed. plan not found"

def test_youbot_gazebo_proxy_exec():
    ygp = robotproxy.YoubotGazeboProxy("test_ygp", 1)
    ygp.load_control_plan(path_to_dict_yaml, path_to_cmds_yaml)    
    ygp.control_loop()






