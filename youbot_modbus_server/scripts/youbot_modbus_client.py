#!/usr/bin/env python

# Author: Tim Zimmerman (timothy.zimmerman@nist.gov)
# Organization: National Institute of Standards and Technology
# U.S. Department of Commerce
# License: Public Domain
#
# Description:  Used for testing the services and messages. Running this 
#               file using the 'rosrun' command will send two sequential
#               requests to the youbot_modbus_server's services (station 
#               status and button status) and print the returned data.
# 

import rospy
import std_msgs.msg
from youbot_modbus_server.srv import *

def poll_modbus_sensor_client():
    print "Waiting for service..."
    rospy.wait_for_service('youbot_modbus_server/get_station_status')
    print "[DONE]"
    try:
        modbus_handler = rospy.ServiceProxy('youbot_modbus_server/get_station_status', YoubotModbusSensorMsg)
        # Build the ROS header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "youbot_modbus_station_request"
        print "\nSent: \n" + str(header)
        return modbus_handler(header)
        
    except rospy.ServiceException, e:
        print "service call failed: " + str(e)
        
def poll_modbus_button_client():
    rospy.wait_for_service('youbot_modbus_server/get_button_status')
    try:
        modbus_handler = rospy.ServiceProxy('youbot_modbus_server/get_button_status', YoubotModbusButtonMsg)
        # Build the ROS header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "youbot_modbus_button_request"
        print "\nSent: \n" + str(header)
        return modbus_handler(header)
        
    except rospy.ServiceException, e:
        print "service call failed: " + str(e)
        
if __name__ == "__main__":
    rospy.init_node('youbot_modbus_client')
    print "\nReceived: \n" + str(poll_modbus_sensor_client())
    print "\n\nReceived: \n" + str(poll_modbus_button_client())