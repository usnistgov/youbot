#!/usr/bin/env python

'''
 @author: Rick Candell
 @contact: rick.candell@nist.gov
 @organization: NIST
'''

import sys
import copy
import rospy
import math
import csv
import geometry_msgs.msg
import my_youbot_msgs.srv

class PlanningInstruction(object):
  prewait_secs = 0.
  postwait_sec = 0.
  def __init__(self, x, y, z, r, p, y):
    return None

class YoubotPlanningServer(object):
  
  _instructions = []  # list of PlanningInstructions
   
  def __init__(self, pserv_name):
      self._pserv_name = pserv_name
      
      # initialize the node
      
      rospy.init_node(self._pserv_name)
      rospy.loginfo("node intialized")
      
      # create the service
      self._s = rospy.Service("get_next_instruction", my_youbot_msgs.srv.PickAndPlace, self.get_next_instruction)
      rospy.loginfo("service created: " + self._pserv_name)
      
      # load the instructions file 
      db_path = rospy.get_param(self._pserv_name+"/db_path")
      rospy.loginfo("loading database: " + db_path)
      if len(db_path) > 0:
        self.load_db(db_path)
      rospy.loginfo("loading database complete")
      
      # wait for the service requests
      rospy.spin()
  
  def get_next_instruction(self):
    nextCmd = my_youbot_pserv.srv.PickAndPlace()
    return nextCmd
    
  def load_instructions(self, path_to_file):
    with open(path_to_file,'r') as f:
        next(f) # skip headings
        reader=csv.reader(f,delimiter='\t')
        for name,age in reader:
            names.append(name)
            ages.append(age) 
    return True


# 
# This is the main subroutine
#  
if __name__ == "__main__":
  if len(sys.argv) == 2:
    print "argv0: " + sys.argv[0]
    print "argv1: " + sys.argv[1]
  else:
      print "usage: planning_server.py server_name"
      print "path to instructions yaml file stored as rosparam /<server_name>/db_path"    
      print "using launch file to set parameter"
      sys.exit(1)
  
  try:
    YoubotPlanningServer(sys.argv[1])
  except rospy.ROSInterruptException:
    pass
