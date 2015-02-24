
'''
@author: Rick Candell
@contact: rick.candell@nist.gov
@organization: NIST
@license: public domain
'''

import rospy
from youbot_msgs.msg import UpdateDependency

class ProxyDepends(object):
    
    def __init__(self, arm_num):
        self._arm_id = "/arm_" + str(arm_num)
        self._d = {}
        rospy.loginfo("created " + self._arm_id)
        self._sub = rospy.Subscriber("robot_depends", UpdateDependency, self.receive_update_depend_cb)
        rospy.loginfo("subscriber created")
        self._pub = rospy.Publisher("robot_depends", UpdateDependency, queue_size=10)
        rospy.loginfo("publisher created")
        
    def transmit_all_update_depend(self):
        for name in self._d.keys():
            self.transmit_update_depend(name, self._d[name])
                    
    def transmit_update_depend(self, name, value):
        msg = UpdateDependency(name, value)
        self._pub.publish(msg)
        rospy.loginfo("transmit dependency update " + name + " with " + str(value))
        
        
    def reset_database(self):
        self._d = {}
        rospy.loginfo("proxy-depends database reset for arm: " + str(self._arm_id))
        
    def receive_update_depend_cb(self, data):
        '''
        @param data: An UpdateDependency object 
        '''
        self._d[data.name] = data.status
        rospy.loginfo("received dependency update " + data.name + " with " + str(data.status))
            
    def wait_for_depend(self, name, timeout_secs=6000):
        rospy.loginfo("waiting for dependency " + name + " with timeout " + str(timeout_secs))
        r = rospy.Rate(10)
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            td = rospy.get_time() - t0
            if td < timeout_secs:
                try:
                    if self._d.has_key(name):
                        if self._d[name] == True:
                            rospy.loginfo("dependency "  +name + " satisfied")
                            break
                        r.sleep()
                except Exception as e:
                    rospy.loginfo("dependency dictionary lookup error") 
                    raise e
            else:
                rospy.loginfo("dependency timeout after "  + td + " seconds")
                raise Exception("system timeout", "dependency timeout after "  + td + " seconds")
            #self.transmit_all_update_depend() 

