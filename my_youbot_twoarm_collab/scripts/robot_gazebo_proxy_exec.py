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
    arm_id = int(rospy.get_param('~arm_num'))
    ygp = robotproxy.