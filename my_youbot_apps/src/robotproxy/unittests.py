
'''
@author: Rick Candell
@contact: rick.candell@nist.gov
@organization: NIST
@license: public domain
'''

import yaml
import robotproxy

def test_joint_pose_defs():
    f = open('/home/rick/catkin_ws/src/youbot/my_youbot_apps/config/joint_pos_defs.yaml')
    d = yaml.load(f)
    print type(d)   
    print d 
    for x in d.keys():
        print x, d[x] 

def test_commands_list():
    f = open('/home/rick/catkin_ws/src/youbot/my_youbot_apps/config/arm1_commands.yaml')
    d = yaml.load(f)
    print type(d)   
    print d 
    for x in d:
        print type(x)
        print x

def test_youbot_gazebo_proxy(): 
    ygp = robotproxy.YoubotGazeboProxy()
    ygp.initialize_node("test_ygp", 1)
