#!/usr/bin/env python

"""
Copyright (c) 2016, Robotnik Automation
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest("interactive_markers")
import rospy, rospkg
import copy
import os

from visualization_msgs.msg import Marker
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Quaternion

from std_srvs.srv import Empty

STANDARD_SIZE = 0.025
DETECTION_SIZE = 0.075


class ContactSwitch(Marker):
	
	def __init__(self, frame_id, name, namespace, marker_id):
		Marker.__init__(self)
		
		self.header.frame_id = frame_id
		self.ns = namespace
		self.id = marker_id
		
		self.type = Marker.SPHERE
		
		self.detection()
		
		self._publisher = rospy.Publisher('~'+name, Marker, queue_size=10) 
		
	
	def add_marker(self):
		self.action = Marker.ADD
		
		
		pass
	
	def delete_marker(self):
		self.action = Marker.DELETE	
	
	def publish(self):
		
		self._publisher.publish(self)
	
	def no_feedback(self):
		
		self.scale.x = STANDARD_SIZE
		self.scale.y = STANDARD_SIZE
		self.scale.z = STANDARD_SIZE
		self.pose.position.z = 0.0
		self.color.r = 0.5
		self.color.g = 0.5
		self.color.b = 0.5
		self.color.a = 1.0
	
	def no_detection(self):
		
		self.scale.x = STANDARD_SIZE
		self.scale.y = STANDARD_SIZE
		self.scale.z = STANDARD_SIZE
		self.pose.position.z = 0.0
		self.color.r = 0.0
		self.color.g = 1.0
		self.color.b = 0.0
		self.color.a = 1.0
	
	def detection(self):
		
		self.scale.x = DETECTION_SIZE
		self.scale.y = DETECTION_SIZE
		self.scale.z = DETECTION_SIZE
		self.pose.position.z = 0.0
		self.color.r = 1.0
		self.color.g = 0.0
		self.color.b = 0.0
		self.color.a = 1.0


class ContactSwitchManager():
	
	def __init__(self, frame_ids=[], names = []):
		
		self._frame_ids = frame_ids
		self._names = frame_ids
		self._marker_list = []
		self._switches_status = Quaternion()
		
		for i in range(len(frame_ids)):
			m = ContactSwitch(frame_ids[i], names[i], 'test', i)
			self._marker_list.append(m)
			m.add_marker()
			m.publish()
	
	def setSwitchesStatus(self, status):
		self._switches_status = status
		
	## @brief forces the application of changes
	def updateMarkers(self):
		
		for marker in self._marker_list:
			marker.publish()
		
		
