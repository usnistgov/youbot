#!/usr/bin/env python

'''
 @author: Rick Candell
 @contact: rick.candell@nist.gov
 @organization: NIST
'''

import sys
import rospy
import moveit_commander
import std_srvs.srv
    
class PoseSelector(object):
    def __init__(self, arm_num, end_effector_link, output_file_name):
        # initialize objects
        self.arm_num = arm_num
        self.end_effector_link = end_effector_link
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._group = moveit_commander.MoveGroupCommander("manipulator")
        rospy.loginfo("move group handle created")
        
        # output file handle - open
        self._output_file = open(output_file_name, 'w')
        rospy.loginfo("pose selector output file opened")
        rospy.logdebug(self._output_file)
        rospy.loginfo("move group handle created")
        
        # init node
        rospy.init_node("pose_select")
        rospy.loginfo("node" + rospy.get_name() + " initialized")
        
    def __del__(self):
        # close output file
        self._output_file.close()
        rospy.loginfo("output file closed")
        
    def disableMotors(self):
        motorOffProxy = rospy.ServiceProxy("/arm_" + str(self.arm_num) + "/switchOffMotors",std_srvs.srv.Empty)
        result = motorOffProxy()
        rospy.loginfo("Motors disabled" + str(result))
    
    def enableMotors(self):
        motorOnProxy = rospy.ServiceProxy("/arm_" + str(self.arm_num) + "/switchOnMotors",std_srvs.srv.Empty)
        result = motorOnProxy()
        rospy.loginfo("Motors re-enabled" + str(result))    
        
    def selectPoses(self):
        print "This program will record poses to a file"
        print "It is best to have two people work together to record poses"
        print "One to move the robot and one to work the software"
        while not rospy.is_shutdown():
            x = raw_input("Press enter to record pose or type \"quit\": ")
            if not (x == "quit" or x == "q"):
                pose = self._group.get_current_pose(self.end_effector_link)
                rpy = self._group.get_current_rpy(self.end_effector_link)
                print "Current pose is: \n"
                print str(pose)                
                pose_name = raw_input("Specify name for the pose: ")
                self.recordPoseToFile(pose_name, pose, rpy)
                rospy.loginfo("Pose recorded")
            else:
                break
    
    def recordPoseToFile(self, name, pose, rpy):
        self._output_file.write("POSE: " + name + "\n")
        self._output_file.write(str(pose))
        self._output_file.write("\n")
        self._output_file.write("RPY: " + str(rpy))
        self._output_file.write("\n=================================\n")
        self._output_file.write("\n")
        print pose
        self._output_file.flush()
        rospy.loginfo("flushed")
# 
# This is the main subroutine
#  
if __name__=='__main__':
    ps = None
    try:
        rospy.loginfo("program started")
        ps = PoseSelector(1, "gripper_pointer_link", "poselist.txt")
        ps.disableMotors()
        ps.selectPoses()
        #ps.enableMotors()
        moveit_commander.os._exit(0)
    except Exception as e:
        rospy.logerr(str(e))
        moveit_commander.os._exit(0)
        sys.exit()



