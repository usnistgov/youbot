#!/usr/bin/env python

import sys
import rospy
import time
import logging
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointPositions
from std_msgs.msg import String

def initializeLogger(loggerName):
  filename = "/home/youbot/log/" + loggerName + "_" + time.strftime("%d%m%y-%H%M%S") + ".log"
  temp_logger = logging.getLogger(loggerName)
  file_handler = logging.FileHandler(filename, mode="w")
  temp_logger.setLevel(logging.INFO)
  temp_logger.addHandler(file_handler)

def data_log_callback(data, source_topic):
  logger = logging.getLogger("youbot_data_logger")
  now = rospy.get_rostime()
  dataToLog = "-\n" # YAML Document Separator
  dataToLog += "  nist_info: \n    source_topic: "+ source_topic + "\n    log_seconds: " + str(now.secs) + "\n    log_nsecs: " + str(now.nsecs) + "\n"
  dataToLog += "  " + str(data).replace("\n", "\n  ")
  logger.info(dataToLog)

# ***MAIN FUNCTION***
def youbot_position_capture():
  # Create the logger
  initializeLogger("youbot_data_logger")
  # Initialize rospy
  rospy.init_node('youbot_data_logger', anonymous=True)

  # Subscribe to ARM 1 data
  rospy.Subscriber("/arm_1/joint_states", JointState, data_log_callback, "arm1_joint_state")
  #rospy.Subscriber("/arm_1/arm_controller/position_command", JointPositions, data_log_callback, "arm1_joint_pos")
  #rospy.Subscriber("/arm_1/gripper_controller/position_command", JointPositions, data_log_callback, "arm1_grip_pos")
  # Subscribe to ARM 2 data
  #rospy.Subscriber("/arm_2/joint_states", JointState, data_log_callback, "arm2_joint_state")
  #rospy.Subscriber("/arm_2/arm_controller/position_command", JointPositions, data_log_callback, "arm2_joint_pos")
  #rospy.Subscriber("/arm_2/gripper_controller/position_command", JointPositions, data_log_callback, "arm2_grip_pos")
  # Spin forever!
  rospy.spin()

# This is the main subroutine

if __name__=='__main__':
  try:  
    youbot_position_capture()
  except rospy.ROSInterruptException:
    pass
