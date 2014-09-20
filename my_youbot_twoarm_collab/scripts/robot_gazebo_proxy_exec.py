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
    print "NS: ", rospy.get_namespace()
    print "NAME: ", rospy.get_name()
    arm_id_path = '/robot_proxy/arm_num'
    arm_id= int(rospy.get_param(arm_id_path))
    path_to_posedict_yaml = rospy.get_param('/robot_proxy/joint_pose_dict')
    path_to_cmds_yaml = rospy.get_param('/robot_proxy/cmd_seq')
    rospy.loginfo("starting youbot gazebo proxy node")
    rospy.loginfo("arm_num = " + str(arm_id))
    ygp = robotproxy.YoubotGazeboProxy("robot_proxy",arm_id)
    ygp.load_control_plan(path_to_posedict_yaml, path_to_cmds_yaml)
    ygp.control_loop()
