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

	try:
	    ygp = robotproxy.YoubotProxy('robot_proxy')
	    ygp.control_loop()
	except Exception as e:
		print e
		
    
    
    
    
    
