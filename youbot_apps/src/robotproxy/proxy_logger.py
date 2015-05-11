
'''
@author: Rick Candell
@contact: rick.candell@nist.gov
@organization: NIST
@license: public domain
'''

import logging
import datetime
import json

class proxy_logger(object):

	def __init__(self, node_name, file_name):
		self.node_name = node_name
		self.f = open(file_name,'w')

	def loginfo(an_object):
		t = datetime.datetime.now()
		jstr = json.dump([t, self.node_name, an_object], self.f)
		

