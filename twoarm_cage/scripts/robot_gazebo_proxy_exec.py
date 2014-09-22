#!/usr/bin/env python

'''
@author: Rick Candell
@contact: rick.candell@nist.gov
@organization: NIST
@license: public domain
'''

import rospy
import robotproxy 

if __name__=="__main__":

    ygp = robotproxy.YoubotGazeboProxy('robot_proxy')
    path_to_posedict_yaml = rospy.get_param('~joint_pose_dict')
    path_to_cmds_yaml = rospy.get_param('~cmd_seq')
    ygp.load_control_plan(path_to_posedict_yaml, path_to_cmds_yaml)
    ygp.control_loop()
    
    
    
    
    
