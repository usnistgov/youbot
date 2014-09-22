
'''
@author: Rick Candell
@contact: rick.candell@nist.gov
@organization: NIST
@license: public domain
'''

import copy
import rospy
from control_msgs.msg import *
from trajectory_msgs.msg import *

class RobotPublisherProxy(object):
    
    def __init__(self, pub_topic, pub_msg_type, joint_names, pub_rate_hz):
        rospy.logdebug("RobotProxy __init__")
        #self._pub_topic = pub_topic
        self._pub_msg_type = pub_msg_type
        self._joint_names = joint_names
        self._pub = rospy.Publisher(pub_topic, pub_msg_type)
        self._rate = rospy.Rate(pub_rate_hz)
        
    def get_joint_names(self):
        rospy.logdebug("RobotProxy get_joint_names")
        return copy.deepcopy(self._joint_names)
    
    def publish_point(self, point, start_time):
        rospy.logdebug("RobotProxy publish_point")
        rospy.logdebug(start_time)
        jt = self._pub_msg_type()
        jt.header.frame_id = 'base_link'
        #goal = FollowJointTrajectoryGoal()
        jt.joint_names = self._joint_names
        #goal = FollowJointTrajectoryGoal()
        jt.points.append(point)
        while(1):
            rospy.logdebug("publishing point")
            self._pub.publish(jt)
            now = rospy.get_time()
            rospy.logdebug(now)
            tdiff = now - start_time
            if (tdiff >= point.time_from_start.to_sec()):
                rospy.logdebug("break.  go to next point")
                break
            else:
                self._rate.sleep()
                
    def publish_trajectory(self, trajectory):
        rospy.logdebug("RobotProxy publish_trajectory")
        rospy.logdebug(trajectory)
        npts = len(trajectory.points)
        start_time = rospy.get_time()
        for ii in range(0,npts):
            rospy.logdebug("point " +str(ii) )
            point = trajectory.points[ii]
            self.publish_point(point, start_time)        
            
class YoubotArmGazeboProxy(RobotPublisherProxy):
    def __init__(self, pub_topic, pub_msg, joint_names, pub_rate_hz):
        rospy.logdebug("YoubotArmProxy init")
        super(YoubotArmGazeboProxy,self).__init__(pub_topic, pub_msg, joint_names, pub_rate_hz)

    def publish_trajectory(self, trajectory):
        rospy.logdebug("YoubotArmProxy publish_trajectory")
        super(YoubotArmGazeboProxy,self).publish_trajectory(trajectory)
        
        