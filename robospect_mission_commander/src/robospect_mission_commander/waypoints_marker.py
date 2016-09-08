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

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarker, Marker, InteractiveMarkerControl
from interactive_markers.menu_handler import *

import actionlib
from geometry_msgs.msg import Pose2D
from robospect_planner.msg import goal, GoToGoal, GoToAction
from std_srvs.srv import Empty

from robospect_msgs.msg import MissionPoint

DISTANCE_BETWEEN_POINTS = 1.0

## @brief Class to manage  the creation of a Waypoint base on InteractiveMarker
class PointPath(InteractiveMarker):
	
	def __init__(self, frame_id, name, description, is_manager = False, speed = 0.2, mission_point = MissionPoint(), point_description = ''):
		InteractiveMarker.__init__(self)
		
		self.header.frame_id = frame_id
		self.name = name
		self.description = description
		self.point_description = point_description
		self.speed = speed
		self.marker = Marker()
		
		self.is_manager = is_manager

		if is_manager:
			self.marker.type = Marker.CUBE
			self.marker.scale.x = 0.15
			self.marker.scale.y = 0.15
			self.marker.scale.z = 0.6
			self.marker.pose.position.z = 0.3
			self.marker.color.r = 1.0
			self.marker.color.g = 0.0
			self.marker.color.b = 0.0
			self.marker.color.a = 0.7
		else:
			self.marker.type = Marker.CYLINDER
			self.marker.scale.x = 0.1
			self.marker.scale.y = 0.1
			self.marker.scale.z = 0.4
			self.marker.pose.position.z = 0.20
			self.marker.color.r = 0.0
			self.marker.color.g = 1.0
			self.marker.color.b = 0.0
			self.marker.color.a = 0.7
			
		self.marker_control = InteractiveMarkerControl()
		self.marker_control.always_visible = True
		self.marker_control.orientation.w = 1
		self.marker_control.orientation.x = 0
		self.marker_control.orientation.y = 1
		self.marker_control.orientation.z = 0
		self.marker_control.markers.append( self.marker )
		self.marker_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
		
		self.mission_point = mission_point
		
		self.controls.append( self.marker_control )
		
	## @brief method called every time that an interaction is received	
	def processFeedback(self, feedback):
		#p = feedback.pose.position
		self.pose = feedback.pose
		self.mission_point.point.x = self.pose.position.x
		self.mission_point.point.y = self.pose.position.y
		#print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)
		#print '%s: %s'%(feedback.marker_name, self.description)
		#self.description = self.description + '\n' + 'x= %lf, y= %lf'%(self.pose.position.x, self.pose.position.y)
		#if not self.is_manager:
		#	self.description = '%s->%s\nx=%.2lf, y=%.2lf'%(self.name, self.point_description, self.mission_point.point.x, self.mission_point.point.y)
		self.updateDescription(self.mission_point.point.x, self.mission_point.point.y)
		
	def updateDescription(self, x, y):
		if not self.is_manager:
			self.description = '%s->%s\nx=%.2lf, y=%.2lf'%(self.name, self.point_description, x, y)
		


## @brief Manages the creation of waypoints and how to send them to Purepursuit
class PointPathManager(InteractiveMarkerServer):
	
	def __init__(self, name, frame_id):
		InteractiveMarkerServer.__init__(self, name)
		self.list_of_points = []
		self.initial_distance_between_points = DISTANCE_BETWEEN_POINTS
		self.frame_id = frame_id
		self.counter_points = 0
		self.mission_options = ['do_crack_detection', 'do_3D_scan', 'do_stereo_image', 'do_ultrasonic_measurements', 'do_teleop_us']
		self.mission_options_ids = {}
		self.num_of_points_options = ['1', '2', '5', '10']
		self.num_of_points_options_ids = {}
		self.distance_between_points_options = ['1.0', '1.2', '1.4', '1.6']
		self.distance_between_points_optionss_ids = {}
		self.num_of_points_to_add = 1
		# Menu handler to create a menu
		self.menu_handler = MenuHandler()
		
		h_first_entry = self.menu_handler.insert( "Inspection options" )
		for i in self.mission_options:
			entry = self.menu_handler.insert( i, parent=h_first_entry, callback=self.modeCb )
			self.menu_handler.setCheckState( entry, MenuHandler.UNCHECKED)
			self.mission_options_ids[entry] = i
			
		#h_first_entry = self.menu_handler.insert( "Waypoints" )
		#entry = self.menu_handler.insert( "Create New", parent=h_first_entry, callback=self.newPointCB)
		entry_new = self.menu_handler.insert( "Number of waypoints")
		for i in self.num_of_points_options:
			entry = self.menu_handler.insert( i, parent=entry_new, callback=self.setNumOfPointsCB )
			if i == '1':
				self.menu_handler.setCheckState( entry, MenuHandler.CHECKED)
			else:
				self.menu_handler.setCheckState( entry, MenuHandler.UNCHECKED)
			self.num_of_points_options_ids[entry] = i
		
		
		entry_new = self.menu_handler.insert( "Distance between waypoints")
		for i in self.distance_between_points_options:
			entry = self.menu_handler.insert( i, parent=entry_new, callback=self.setDistanceBetweenWaypointsCB )
			if i == '1.0':
				self.menu_handler.setCheckState( entry, MenuHandler.CHECKED)
			else:
				self.menu_handler.setCheckState( entry, MenuHandler.UNCHECKED)
			self.distance_between_points_optionss_ids[entry] = i
		
		
		
		entry_new = self.menu_handler.insert( "Create new waypoint(s)", callback=self.newPointCB)
		entry = self.menu_handler.insert( "Delete last waypoint(s)", callback=self.deleteLastPointCB );
		entry = self.menu_handler.insert( "Delete first waypoint(s)", callback=self.deleteFirstPointCB );
		entry = self.menu_handler.insert( "Delete all waypoints", callback=self.deleteAllPointsCB );
		entry = self.menu_handler.insert( "Print waypoints", callback=self.printAllPointsCB );
		
		#entry = self.menu_handler.insert( "Create", parent=entry_new, callback=self.newPointCB)
		
		h_second_entry = self.menu_handler.insert( "Path" )
		#entry = self.menu_handler.insert( "Go", parent=h_second_entry, callback=self.startRouteCB)	# Send the path from the first point to the last one
		#entry = self.menu_handler.insert( "Stop", parent=h_second_entry, callback=self.stopRouteCB)	# Stops the current path 
		#entry = self.menu_handler.insert( "Go back", parent=h_second_entry, callback=self.reverseRouteCB) # Sends the path from the last point to the first one
		
		# Creates the first point
		#self.list_of_points.append(PointPath(frame_id, 'p1', 'p1'))
		self.initial_point = PointPath(frame_id, 'PointManager', 'PointManager', True)
		self.insert(self.initial_point, self.initial_point.processFeedback)
		
		self.menu_handler.apply( self, self.initial_point.name )
		self.applyChanges()
		
		#self.planner_client = PurepursuitClient(planner)
		
		# Locates and loads the UI file into the widget
		#rp = rospkg.RosPack()		
	
	# @brief callback for checkbox events	
	def modeCb(self, feedback):
		
		h_mode_last = feedback.menu_entry_id
		
		if self.menu_handler.getCheckState( h_mode_last ) == MenuHandler.CHECKED:
			self.menu_handler.setCheckState( h_mode_last, MenuHandler.UNCHECKED )
		else:
			self.menu_handler.setCheckState( h_mode_last, MenuHandler.CHECKED )
			
		#rospy.loginfo("Switching to menu entry #" + self.mission_options_ids[h_mode_last])
		self.menu_handler.reApply( self )
		self.applyChanges()
	
	
	## @brief Creates a new PointPath and save it a list
	def createNewPoint(self, speed = 0.2):
		
		for i in range(self.num_of_points_to_add):
			m_point = MissionPoint()
			m_point.do_crack_detection = m_point.do_3D_scan = m_point.do_stereo_image = m_point.do_ultrasonic_measurements = m_point.do_teleop_us = False
			
			point_description = ''
			for i in self.mission_options_ids:
				if self.menu_handler.getCheckState( i ) == MenuHandler.CHECKED:
					if self.mission_options_ids[i] == 'do_crack_detection':
						m_point.do_crack_detection = True
						point_description += ' crack'
					elif self.mission_options_ids[i] == 'do_3D_scan':
						m_point.do_3D_scan = True
						point_description += ' 3D'
					elif self.mission_options_ids[i] == 'do_stereo_image':
						m_point.do_stereo_image = True
						point_description += ' stereo'
					elif self.mission_options_ids[i] == 'do_ultrasonic_measurements':
						m_point.do_ultrasonic_measurements = True
						point_description += ' us'
					elif self.mission_options_ids[i] == 'do_teleop_us':
						m_point.do_teleop_us = True
						point_description += ' tus'
					
					
			##print 'Creating new point %d'%(self.counter_points)
			new_point = PointPath(self.frame_id, 'p%d'%(self.counter_points), 'p%d->%s'%(self.counter_points, point_description), speed = speed, mission_point = m_point, point_description = point_description)
			
			if len(self.list_of_points) > 1:
				new_point.pose.position.x = self.list_of_points[self.counter_points-1].pose.position.x
				new_point.pose.position.y = self.list_of_points[self.counter_points-1].pose.position.y
			elif len(self.list_of_points) == 1:
				new_point.pose.position.x = self.list_of_points[0].pose.position.x
				new_point.pose.position.y = self.list_of_points[0].pose.position.y
		
			new_point.pose.position.x = new_point.pose.position.x  + self.initial_distance_between_points
			new_point.mission_point.point.x = new_point.pose.position.x
				
			new_point.updateDescription(new_point.pose.position.x, new_point.pose.position.y)
			#print 'Creating new point at position %.2lf, %.2lf, %.2lf'%(new_point.pose.position.x, new_point.pose.position.y, new_point.pose.position.z)
			
			self.list_of_points.append(new_point)
			#dict_of_points[point_name] = new_point
			self.insert(new_point, new_point.processFeedback)
			self.menu_handler.apply( self, 'p%d'%(self.counter_points) )
			self.applyChanges()
			self.counter_points = self.counter_points + 1
		
		#for i in self.list_of_points:
		#	print 'Point %s: %.2lf, %.2lf, %.2lf'%(i.name, i.pose.position.x, i.pose.position.y, i.pose.position.z)
			
		return
	
	
		
	## @brief Callback called to create a new point	
	def newPointCB(self, feedback):
		#print 'newPointCB'
		self.createNewPoint()
	
	## @brief Callback called to set the number of points added every time	
	def setNumOfPointsCB(self, feedback):
		
		for entry in self.num_of_points_options_ids:
			self.menu_handler.setCheckState( entry, MenuHandler.UNCHECKED )
		
		self.menu_handler.setCheckState( feedback.menu_entry_id, MenuHandler.CHECKED ) 
		
		self.num_of_points_to_add = int(self.num_of_points_options_ids[feedback.menu_entry_id])
		
		#print 'points = %d'%self.num_of_points_to_add
	
	
	## @brief Callback called to set distance between points
	def setDistanceBetweenWaypointsCB(self, feedback):
		
		for entry in self.distance_between_points_optionss_ids:
			self.menu_handler.setCheckState( entry, MenuHandler.UNCHECKED )
		
		self.menu_handler.setCheckState( feedback.menu_entry_id, MenuHandler.CHECKED ) 
		
		self.initial_distance_between_points = float(self.distance_between_points_optionss_ids[feedback.menu_entry_id])
		
		#print 'points = %d'%self.num_of_points_to_add
		
	## @brief Callback called to create a new poing	
	def deleteLastPointCB(self, feedback):
		for i in range(self.num_of_points_to_add):
			if self.counter_points > 0:
				 p = self.list_of_points.pop()
				 self.counter_points = self.counter_points - 1
				 self.erase(p.name)
				 self.applyChanges()
			else:
				break
			 
		 #print 'deleteLastPointCB'	
	
	## @brief Callback called to create a new poing	
	def deleteFirstPointCB(self, feedback):
		
		"""if self.num_of_points_to_add > self.counter_points:
			self.deleteAllPoints()
		else:
			new_list = []
			diff_length = self.counter_points - self.num_of_points_to_add
			
			for i in range(self.counter_points-diff_length):
				new_list.append(self.list_of_points[i + diff_length])
				self.erase(self.list_of_points[i].name)
			
			#print new_list
			self.counter_points = self.counter_points - diff_length
			self.list_of_points = new_list
			#self.erase(p.name)
			self.applyChanges()
			
		"""
		pass
		 #print 'deleteLastPointCB'	
	
	## @brief Function called to delete all the waypoints
	def deleteAllPoints(self):
		for i in range(len(self.list_of_points)):
			p = self.list_of_points.pop()
			self.counter_points = self.counter_points - 1
			self.erase(p.name)
			 
		self.applyChanges()
		
	## @brief Callback called to delete all the waypoints
	def deleteAllPointsCB(self, feedback):
		self.deleteAllPoints()
	
	## @brief Callback called to print all the waypoints
	def printAllPointsCB(self, feedback):
		print self.convertListOfPointPathIntoGoal()
	
	
	## @brief Starts the route
	def startRouteCB(self, feedback):
		goals = self.convertListOfPointPathIntoGoal()

		return
	
	## @brief Starts the route on the inverse direction
	def reverseRouteCB(self, feedback):
		goals = self.convertListOfPointPathIntoGoal(inverse = True)
		return
		
	## @brief Stops the current route if it's started
	def stopRouteCB(self, feedback):
		#self.planner_client.cancel()
		return
	
	## @brief Starts the route (inverse order of waypoints)
	def convertListOfPointPathIntoGoal(self, inverse = False):
		converted_list = []
		if inverse:
			for i in reversed(self.list_of_points):
				converted_list.append(goal(pose = Pose2D(i.pose.position.x, i.pose.position.y, 0.0), speed = i.speed)) # For now speed constant
		else:
			for i in self.list_of_points:
				#print 'convertListOfPointPathIntoGoal: %.2lf, %.2lf'%(i.pose.position.x, i.pose.position.y)
				converted_list.append(goal(pose = Pose2D(i.pose.position.x, i.pose.position.y, 0.0), speed = i.speed)) # For now speed constant
		
		return converted_list
	
	## @brief Fake service to emulate the event start route
	def goService(self, param):
		rospy.loginfo('%s::goService'%(rospy.get_name()))
		
		self.startRouteCB(None)
		
		return []
	
	## @brief Fake service to emulate the event reverse route
	def goBackService(self, param):
		rospy.loginfo('%s::goBackService'%(rospy.get_name()))
		
		self.reverseRouteCB(None)
		
		return []
		
	## @brief Fake service to emulate the event cancel route
	def cancelService(self, param):
		rospy.loginfo('%s::cancelService'%(rospy.get_name()))
		self.stopRouteCB(None)
		
		return []
	
	
	# @brief Returns the list of waypoints
	def getMissionPoints(self):
	
		converted_list = []
		for i in self.list_of_points:
			converted_list.append(i.mission_point) 
		
		return converted_list
		
	#@brief gets the number of points
	def getSizeMissionPoints(self):
		return len(self.list_of_points)
		
	## @brief forces the application of changes
	def updateMarkers(self):
		self.menu_handler.reApply( self )
		self.applyChanges()
