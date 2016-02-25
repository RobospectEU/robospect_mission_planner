#!/usr/bin/env python

# Copyright (c) 2015, Robotnik Automation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import rospkg
import threading

import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot, QBasicTimer, SIGNAL
from python_qt_binding.QtGui import QKeySequence, QShortcut, QWidget, QPixmap, QMessageBox, QStandardItemModel,QStandardItem
from rqt_gui_py.plugin import Plugin
#from qt_gui.plugin import Plugin

import time
import math

from std_msgs.msg import String
from robospect_msgs.msg import MissionCommand, MissionState
from robospect_msgs.msg import State as RobospectState
from robospect_msgs.srv import MissionCommandSrv
from robotnik_msgs.msg import State
from robospect_planner.msg import State as PlannerState
from robotnik_trajectory_planner.msg import State as TrajectoryPlannerState
from tf import TransformListener, Exception as tfException, ConnectivityException, LookupException, ExtrapolationException

from rospy.exceptions import ROSException

from waypoints_marker import *

TIMEOUT_SERVICE = 2.0
TIMEOUT_TOPIC = 2.0
mission_commands = ['cracks', 'distance', 'threshold', 'stop', 'waypoints']


class MissionCommanderGUI(Plugin):
	
	def __init__(self, context):
		super(MissionCommanderGUI, self).__init__(context)
		self.setObjectName('MissionCommanderGUI')

		self._widget = QWidget()
			
		rp = rospkg.RosPack()
		
		self.state_string = " "
		
		# UI
		ui_file = os.path.join(rp.get_path('robospect_mission_commander'), 'resource', 'MissionCommander_v1.ui')
		loadUi(ui_file, self._widget)
		self._widget.setObjectName('MissionCommanderGUI')
		
		pixmap_red_file = os.path.join(rp.get_path('robospect_mission_commander'), 'resource', 'red.png')
		pixmap_green_file = os.path.join(rp.get_path('robospect_mission_commander'), 'resource', 'green.png')
		self._pixmap_red = QPixmap(pixmap_red_file)
		self._pixmap_green = QPixmap(pixmap_green_file)
		
		self._widget.label_platform_controller_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_platform_navigation_planner_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_platform_localization_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_platform_trajectory_planner_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_mission_state.setPixmap(self._pixmap_red) # Shows connection  state
		
		for command in mission_commands:
			self._widget.comboBox_command.addItem(command)
					
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))		
		
		# Adds this widget to the context
		context.add_widget(self._widget)
	
		
		self._mission_state_topic_connected = False
		
		# Inits connections and gui components values
		self._init()
		
		# HANDLERS
		# Adds handlers to 'press button' event
		
		self._widget.pushButton_send.clicked.connect(self._send_comand_cb)
		self._update_timer_interface = 200 # Frequency to update values in the interface (ms)
		
		
		self.waypoints_markers = PointPathManager('mission_waypoints', frame_id = '/map')
		
		
		self._init_timers()
	
	
	def _init(self):
		'''
			Inits topic names, topic and service connections
		'''
		self._command_service = rospy.get_param('/services/mission_command', default='/mission_command')
		self._mission_state_topic = rospy.get_param('/topics/mission_state', default='/mission_state')
		self._platform_controller_state_topic = rospy.get_param('/topics/platform_controller_state', default='/robospect_platform_controller/state')
		self._navigation_planner_state_topic = rospy.get_param('/topics/navigation_planner_state', default='/robospect_planner/state')
		self._localization_state_topic = rospy.get_param('/topics/localization_state', default='/nav200_laser_node/state')
		self._trajectory_planner_state_topic = rospy.get_param('/topics/trajectory_planner_state', default='/rt_traj_planner/state')
		self._trajectory_control_state_topic = rospy.get_param('/topics/trajectory_control_state', default='/rt_traj_exec/state')
		#self._map_frame_id =  = rospy.get_param('/global_params/map_frame_id', default='/map')
		#self._base_frame_id =  = rospy.get_param('/global_params/base_frame_id', default='/base_footprint')
		
		# Attribute to save the current mission state
		self._components_state = {}
		self._components_state['mission_state'] = {'state': MissionState(), 'time': rospy.Time.now(), 'label': self._widget.label_mission_state}
		self._components_state['platform_controller_state'] = {'state': RobospectState(), 'time': rospy.Time.now(), 'label': self._widget.label_platform_controller_state}
		self._components_state['navigation_planner_state'] = {'state': State(), 'time': rospy.Time.now(), 'label': self._widget.label_platform_navigation_planner_state}
		self._components_state['localization_state'] = {'state': State(), 'time': rospy.Time.now(), 'label': self._widget.label_platform_localization_state}
		self._components_state['trajectory_planner_state'] = {'state': State(), 'time': rospy.Time.now(), 'label': self._widget.label_platform_trajectory_planner_state}
		self._components_state['trajectory_control_state'] = {'state': State(), 'time': rospy.Time.now(), 'label': None}
		
		
		# SUBSCRIPTIONS
		self._mission_state_sub = rospy.Subscriber(self._mission_state_topic, MissionState, self._mission_state_cb, queue_size = 10)
		self._platform_controller_state_sub = rospy.Subscriber(self._platform_controller_state_topic, RobospectState, self._platform_controller_state_cb, queue_size=10)
		self._navigation_planner_state_sub = rospy.Subscriber(self._navigation_planner_state_topic, PlannerState, self._navigation_planner_state_cb, queue_size=10)
		self._localization_state_sub = rospy.Subscriber(self._localization_state_topic, State, self._localization_state_cb, queue_size=10)
		self._trajectory_planner_state_sub = rospy.Subscriber(self._trajectory_planner_state_topic, TrajectoryPlannerState, self._trajectory_planner_state_cb, queue_size=10)
		self._trajectory_control_state_sub = rospy.Subscriber(self._trajectory_control_state_topic, State, self._trajectory_control_state_cb, queue_size=10)
		
		# transform listener to look for transformations
		#self._transform_listener = TransformListener()
		
		# PUBLICATIONS
		'''
		if hasattr(self, ''):
			self..unregister()
			rospy.loginfo('Unregistering from command publisher')
		try:
			self. = rospy.Publisher(self._command_service, MissionCommand, queue_size=10)
		except ROSException, e:
			rospy.logerr('MissionCommanderGUI: Error creating publisher for topic %s (%s)'%(self._command_service, e))
		'''
		self._command_service_client = rospy.ServiceProxy(self._command_service, MissionCommandSrv)
		
			
			
	def _init_timers(self):
		'''
			inits the timers used to control the connection, ..
		'''
		# Creates a basic timer and starts it
		self._timer = QBasicTimer()
		self._timer.start(self._update_timer_interface, self)
		
	
	def _send_comand_cb(self):
		'''
			Sends a mission command
		'''
		msg = MissionCommand()	
		msg.command = self._widget.comboBox_command.currentText()
		
		try:
			msg.variable = float(self._widget.lineEdit_parameter.text().encode("utf8"))
		except ValueError, e:
			rospy.logerr('MissionCommanderGUI:_send_command_cb: %s',e)
			QMessageBox.warning(self._widget, 'Error', 'Incorrect format of the parameter. A number is expected')
					
		srv = MissionCommandSrv()
		
		if msg.command == 'waypoints':
			waypoints = self.waypoints_markers.getMissionPoints()
			
			if len(waypoints) == 0:
				QMessageBox.warning(self._widget, 'Error', 'No waypoints to send')
				return
			
			msg.points = waypoints
		
		#srv.command = msg
		
		try:
			self._command_service_client(msg)
		except rospy.ROSInterruptException,e: 
			rospy.logerr('MissionCommanderGUI:_send_command_cb: %s',e)
			QMessageBox.warning(self._widget, 'Error', 'Error sending the command')
		except rospy.ServiceException,e: 
			rospy.logerr('MissionCommanderGUI:_send_command_cb: %s',e)
			QMessageBox.warning(self._widget, 'Error', 'Error sending the command')
				
	
	def shutdown_plugin(self):
		'''
			Shutdowns the component
		'''
		self._mission_state_sub.unregister()
		
		
	def _mission_state_cb(self, msg):
		'''
			Receives the state of the mission
		'''
		self._components_state['mission_state']['state'] = msg
		self._components_state['mission_state']['time'] = rospy.Time.now()
	
		
	def _platform_controller_state_cb(self, msg):
		'''
			Receives the state of the platform controller
		'''
		self._components_state['platform_controller_state']['state'] = msg
		self._components_state['platform_controller_state']['time'] = rospy.Time.now()
	
		
	def _navigation_planner_state_cb(self, msg):
		'''
			Receives the state of the navigation planner
		'''
		self._components_state['navigation_planner_state']['state'] = msg
		self._components_state['navigation_planner_state']['time'] = rospy.Time.now()
		
		
	def _localization_state_cb(self):
		'''
			checks the state of the localization node
		'''
		self._components_state['localization_state']['state'] = msg
		self._components_state['localization_state']['time'] = rospy.Time.now()
		
		
	def _trajectory_planner_state_cb(self, msg):
		'''
			Receives the state of the trajectory planner
		'''
		self._components_state['trajectory_planner_state']['state'] = msg
		self._components_state['trajectory_planner_state']['time'] = rospy.Time.now()
		
	def _trajectory_control_state_cb(self, msg):
		'''
			Receives the state of the trajectory control
		'''
		self._components_state['trajectory_control_state']['state'] = msg
		self._components_state['trajectory_control_state']['time'] = rospy.Time.now()
	
	
	def timerEvent(self, e):
		'''
			Method executed periodically
			Updates the graphical qt components
		'''
		
		
		# Updating Mission State
		
		try:
			self._widget.lineEdit_mission_state.setText('%s'%self._components_state['mission_state']['state'].mission_state)
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		
		
		# Mission waypoints
		try:
			self._widget.lineEdit_waypoints.setText('%d'%self.waypoints_markers.getSizeMissionPoints())
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		
		self.waypoints_markers.updateMarkers()
		
		
		# Battery
		try:
			self._widget.progressBar_battery.setValue(int(self._components_state['mission_state']['state'].vehicle_state.battery_level))
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		
		# Vehicle
		try:
			self._widget.lineEdit_vehicle_command.setText('%s'%self._components_state['mission_state']['state'].vehicle_state.command) # command 
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		
		try:
			self._widget.lineEdit_vehicle_state.setText(self._components_state['mission_state']['state'].vehicle_state.state)	# State
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_base_x.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.vehicle_x)	#X base
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_base_y.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.vehicle_y)	#Y base 
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_base_theta.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.vehicle_x)	#THETA base
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_base_velocity.setText('%.1f,%.1f'%(self._components_state['mission_state']['state'].vehicle_state.vehicle_linear_speed, self._components_state['mission_state']['state'].vehicle_state.vehicle_angular_speed))	#linear speed
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		
		try:
			self._widget.lineEdit_crane_x.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_x)	#X crane
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_crane_y.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_y)	#Y crane 
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_crane_z.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_z)	#Z crane
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_crane_q1.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_q1)	#q1 crane
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_crane_q2.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_q2)	#q2 crane 
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_crane_q3.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_q3)	#q3 crane
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_crane_q4.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_q4)	#q4 crane
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		
		if len(self._components_state['mission_state']['state'].vehicle_state.crane_joints) == 7:
			
			try:
				self._widget.lineEdit_crane_j1.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_joints[0])	#j1 crane
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
			try:
				self._widget.lineEdit_crane_j2.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_joints[1])	#j2 crane
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
			try:
				self._widget.lineEdit_crane_j3.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_joints[2])	#j3 crane
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
			try:
				self._widget.lineEdit_crane_j4.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_joints[3])	#j4 crane
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
			try:
				self._widget.lineEdit_crane_j5.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_joints[4])	#j5 crane
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
			try:
				self._widget.lineEdit_crane_j6.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_joints[5])	#j6 crane
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
			try:
				self._widget.lineEdit_crane_j7.setText('%.3f'%self._components_state['mission_state']['state'].vehicle_state.crane_joints[6])	#j7 crane
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
		
		
		t_now = rospy.Time.now()
		for component in self._components_state:
			if self._components_state[component]['label']:
				if (t_now - self._components_state[component]['time']).to_sec() > TIMEOUT_TOPIC:
					self._components_state[component]['label'].setPixmap(self._pixmap_red)
				else:
					self._components_state[component]['label'].setPixmap(self._pixmap_green)
			
		"""# Checks the ROS connection
		t = time.time()
		if self._topic_connected and (t - self._topic_timer >= self._topic_timeout_connection):
			self._topic_connected = False
			rospy.logerr('PowerballGUI: error in communication with %s'%self._topic)
			self._widget.qlabel_state_connection.setPixmap(self._pixmap_red)
		
		if self._topic_connected:
			self._widget.qlabel_state_connection.setPixmap(self._pixmap_green)
        
		if self._topic_joint_states_connected and (t - self._topic_joint_states_timer >= self._topic_timeout_connection):
			self._topic_joint_states_connected = False
			rospy.logerr('PowerballGUI: error in communication with %s'%self._joint_states_topic)
			self._widget.qlabel_joint_states_connection.setPixmap(self._pixmap_red)
		
		if self._topic_joint_states_connected:
			self._widget.qlabel_joint_states_connection.setPixmap(self._pixmap_green)
        """
        
