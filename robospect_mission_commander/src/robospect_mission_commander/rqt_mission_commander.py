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

from std_msgs.msg import String, Bool
from robospect_msgs.msg import MissionCommand, MissionState, PlatformCommand, PadStatus
from robospect_msgs.msg import State as RobospectState
from robospect_msgs.srv import MissionCommandSrv, SetControlMode, PlatformCommandSrv
from robotnik_msgs.msg import State
from robospect_planner.msg import State as PlannerState
from robotnik_trajectory_planner.msg import State as TrajectoryPlannerState
from tf import TransformListener, Exception as tfException, ConnectivityException, LookupException, ExtrapolationException
from nav200_laser.msg import Pose2D_nav
from dynamixel_msgs.msg import JointState as DynamixelState
from std_srvs.srv import Empty
from rospy.exceptions import ROSException

from waypoints_marker import *
from geometry_msgs.msg import Quaternion

TIMEOUT_SERVICE = 2.0
TIMEOUT_TOPIC = 2.0
mission_commands = ['cracks', 'distance', 'threshold', 'stop', 'complete', 'complete_offline']
control_modes = ['POSITION', 'VELOCITY']
NAV200_MIN_QUALITY = 30


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
		pixmap_orange_file = os.path.join(rp.get_path('robospect_mission_commander'), 'resource', 'orange.png')
		self._pixmap_red = QPixmap(pixmap_red_file)
		self._pixmap_green = QPixmap(pixmap_green_file)
		self._pixmap_orange = QPixmap(pixmap_orange_file)
		
		self._widget.label_platform_controller_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_platform_navigation_planner_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_platform_localization_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_platform_trajectory_planner_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_mission_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_pad_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_pad_vehicle_control.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_pad_crane_control.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_pan_motor_moving.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_switches_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_sw1_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_sw2_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_sw3_state.setPixmap(self._pixmap_red) # Shows connection  state
		self._widget.label_sw4_state.setPixmap(self._pixmap_red) # Shows connection  state
		
		for command in mission_commands:
			self._widget.comboBox_command.addItem(command)
		for mode in control_modes:
			self._widget.comboBox_control_mode.addItem(mode)
					
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))		
		
		# Adds this widget to the context
		context.add_widget(self._widget)
	
		
		self._mission_state_topic_connected = False
		
		# Inits connections and gui components values
		self._init()
		
		# HANDLERS
		# Adds handlers to 'press button' event
		
		self._widget.pushButton_initialize_platform.clicked.connect(self._initialize_platform_cb)
		self._widget.pushButton_reset_steering_encoder.clicked.connect(self._reset_steering_encoder_cb)
		self._widget.pushButton_send.clicked.connect(self._send_comand_cb)
		self._widget.pushButton_set_control_mode.clicked.connect(self._set_control_mode_cb)
		self._widget.pushButton_fold_crane.clicked.connect(self._fold_crane_cb)
		self._widget.pushButton_stop_vehicle_action.clicked.connect(self._stop_vehicle_cb)
		self._widget.pushButton_set_pantilt.clicked.connect(self._set_pantilt_cb)
		self._widget.pushButton_stop_teleop.clicked.connect(self._stop_teleop_cb)
		self._update_timer_interface = 100 # Frequency to update values in the interface (ms)
		
		
		self.waypoints_markers = PointPathManager('mission_waypoints', frame_id = '/map')
		
		
		self._init_timers()
	
	
	def _init(self):
		'''
			Inits topic names, topic and service connections
		'''
		self._command_service = rospy.get_param('/services/mission_command', default='/mission_command_srv')
		self._init_platform_service = rospy.get_param('/services/initialize_platform', default='/robospect_platform_controller/initialize_modbus_controller')
		self._reset_steering_encoder_service = rospy.get_param('/services/reset_steering_encoder', default='/robospect_platform_controller/reset_steering_encoder')
		self._set_control_mode_service = rospy.get_param('/services/set_control_mode', default='/robospect_platform_controller/set_control_mode')
		self._platform_service = rospy.get_param('/services/platform_command', default='/platform_command_srv')
		
		self._mission_state_topic = rospy.get_param('/topics/mission_state', default='/mission_state')
		self._platform_controller_state_topic = rospy.get_param('/topics/platform_controller_state', default='/robospect_platform_controller/state')
		self._navigation_planner_state_topic = rospy.get_param('/topics/navigation_planner_state', default='/robospect_planner/state')
		self._localization_state_topic = rospy.get_param('/topics/localization_state', default='/nav200_laser_node/nav200_pose')
		self._trajectory_planner_state_topic = rospy.get_param('/topics/trajectory_planner_state', default='/rt_traj_planner/state')
		self._trajectory_control_state_topic = rospy.get_param('/topics/trajectory_control_state', default='/rt_traj_exec/state')
		self._pan_state_topic = rospy.get_param('/topics/pan_state_topic', default='/pan_controller/state')
		self._tilt_state_topic = rospy.get_param('/topics/tilt_state_topic', default='/tilt_controller/state')
		self._pad_state_topic = rospy.get_param('/topics/pad_state_topic', default='/robospect_pad/state')
		self._teleop_done_topic = rospy.get_param('/topics/teleop_done', default='/teleop_done')
		self._contact_switches_topic = rospy.get_param('/topics/contact_switches_topic', default='/contact_switches_state')
		#self._map_frame_id =  = rospy.get_param('/global_params/map_frame_id', default='/map')
		#self._base_frame_id =  = rospy.get_param('/global_params/base_frame_id', default='/base_footprint')
		
		# Attribute to save the current mission state
		self._components_state = {}
		self._components_state['mission_state'] = {'state': MissionState(), 'time': rospy.Time.now(), 'label': self._widget.label_mission_state}
		self._components_state['platform_controller_state'] = {'state': RobospectState(), 'time': rospy.Time.now(), 'label': self._widget.label_platform_controller_state}
		self._components_state['navigation_planner_state'] = {'state': State(), 'time': rospy.Time.now(), 'label': self._widget.label_platform_navigation_planner_state}
		self._components_state['localization_state'] = {'state': Pose2D_nav(), 'time': rospy.Time.now(), 'label': self._widget.label_platform_localization_state}
		self._components_state['trajectory_planner_state'] = {'state': State(), 'time': rospy.Time.now(), 'label': self._widget.label_platform_trajectory_planner_state}
		self._components_state['pan_state'] = {'state': DynamixelState(), 'time': rospy.Time.now(), 'label': self._widget.label_pan_state, 'label_moving': self._widget.label_pan_motor_moving}
		self._components_state['tilt_state'] = {'state': DynamixelState(), 'time': rospy.Time.now(), 'label': self._widget.label_tilt_state, 'label_moving': self._widget.label_tilt_motor_moving}
		self._components_state['trajectory_control_state'] = {'state': State(), 'time': rospy.Time.now(), 'label': None}
		self._components_state['pad_state'] = {'state': PadStatus(), 'time': rospy.Time.now(), 'label': self._widget.label_pad_state, 'label_vehicle': self._widget.label_pad_vehicle_control, 'label_crane': self._widget.label_pad_crane_control}
		self._components_state['contact_switches_state'] = {'state': Quaternion(), 'time': rospy.Time.now(), 'label': self._widget.label_switches_state, 'label_sw1': self._widget.label_sw1_state, 'label_sw2': self._widget.label_sw2_state,'label_sw3': self._widget.label_sw3_state,'label_sw4': self._widget.label_sw4_state, }
		
		
		# SUBSCRIPTIONS
		self._mission_state_sub = rospy.Subscriber(self._mission_state_topic, MissionState, self._mission_state_cb, queue_size = 10)
		self._platform_controller_state_sub = rospy.Subscriber(self._platform_controller_state_topic, RobospectState, self._platform_controller_state_cb, queue_size=10)
		self._navigation_planner_state_sub = rospy.Subscriber(self._navigation_planner_state_topic, PlannerState, self._navigation_planner_state_cb, queue_size=10)
		self._localization_state_sub = rospy.Subscriber(self._localization_state_topic, Pose2D_nav, self._localization_state_cb, queue_size=10)
		self._trajectory_planner_state_sub = rospy.Subscriber(self._trajectory_planner_state_topic, TrajectoryPlannerState, self._trajectory_planner_state_cb, queue_size=10)
		self._trajectory_control_state_sub = rospy.Subscriber(self._trajectory_control_state_topic, State, self._trajectory_control_state_cb, queue_size=10)
		self._pan_state_sub = rospy.Subscriber(self._pan_state_topic, DynamixelState, self._pan_state_cb, queue_size = 2)
		self._tilt_state_sub = rospy.Subscriber(self._tilt_state_topic, DynamixelState, self._tilt_state_cb, queue_size = 2)
		self._pad_state_sub = rospy.Subscriber(self._pad_state_topic, PadStatus, self._pad_state_cb, queue_size = 2)
		self._contact_switch_state_sub = rospy.Subscriber(self._contact_switches_topic, Quaternion, self._contact_switches_state_cb, queue_size = 2)
		
		# transform listener to look for transformations
		#self._transform_listener = TransformListener()
		
		# PUBLICATIONS
		self.teleop_done_pub = rospy.Publisher(self._teleop_done_topic, Bool, queue_size=10) 
		'''
		if hasattr(self, ''):
			self..unregister()
			rospy.loginfo('Unregistering from command publisher')
		try:
			self. = rospy.Publisher(self._command_service, MissionCommand, queue_size=10)
		except ROSException, e:
			rospy.logerr('MissionCommanderGUI: Error creating publisher for topic %s (%s)'%(self._command_service, e))
		'''
		# Services
		self._command_service_client = rospy.ServiceProxy(self._command_service, MissionCommandSrv)
		self._initialize_platform_service_client = rospy.ServiceProxy(self._init_platform_service, Empty)
		self._reset_steering_encoder_service_client = rospy.ServiceProxy(self._reset_steering_encoder_service, Empty)
		self._set_control_mode_service_client = rospy.ServiceProxy(self._set_control_mode_service, SetControlMode)
		self._platform_command_service_client = rospy.ServiceProxy(self._platform_service, PlatformCommandSrv)
		
			
			
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
		
		if msg.command == 'complete' or msg.command == 'complete_offline':
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
	
	def _fold_crane_cb(self):
		'''
			Sends the foldCrane command to the platform
		'''
		msg = PlatformCommand()	
		msg.command = 'foldCrane'
		
		ret = QMessageBox.question(self._widget, "Fold Crane", 'Do you want to fold the crane?', QMessageBox.Ok, QMessageBox.Cancel)
		
		if ret == QMessageBox.Ok:
			try:
				self._platform_command_service_client(msg)
			except rospy.ROSInterruptException,e: 
				rospy.logerr('MissionCommanderGUI:_fold_crane_cb: %s',e)
				QMessageBox.warning(self._widget, 'Error', 'Error sending the command')
			except rospy.ServiceException,e: 
				rospy.logerr('MissionCommanderGUI:_fold_crane_cb: %s',e)
				QMessageBox.warning(self._widget, 'Error', 'Error sending the command')
	
	
	def _set_pantilt_cb(self):
		'''
			Sends the setPanTilt command to the platform
		'''
		msg = PlatformCommand()	
		msg.command = 'setPanTilt'
		
		msg.variables = [self._widget.doubleSpinBox_set_pan.value(), self._widget.doubleSpinBox_set_tilt.value()]
		#msg.variables = [0.0, 0.0]
		
		#print self._widget.doubleSpinBox_set_pan.value()
		
		try:
			self._platform_command_service_client(msg)
		except rospy.ROSInterruptException,e: 
			rospy.logerr('MissionCommanderGUI:_set_pantilt_cb: %s',e)
			QMessageBox.warning(self._widget, 'Error', 'Error sending the command')
		except rospy.ServiceException,e: 
			rospy.logerr('MissionCommanderGUI:_set_pantilt_cb: %s',e)
			QMessageBox.warning(self._widget, 'Error', 'Error sending the command')
			
				
	def _stop_vehicle_cb(self):
		'''
			Sends the stopcommand to the platform
		'''
		msg = PlatformCommand()	
		msg.command = 'cancel'
		
		ret = QMessageBox.question(self._widget, "Stop Vehicle", 'Do you want to stop the vehicle?', QMessageBox.Ok, QMessageBox.Cancel)
		
		if ret == QMessageBox.Ok:
			try:
				self._platform_command_service_client(msg)
			except rospy.ROSInterruptException,e: 
				rospy.logerr('MissionCommanderGUI:_stop_vehicle_cb: %s',e)
				QMessageBox.warning(self._widget, 'Error', 'Error sending the command')
			except rospy.ServiceException,e: 
				rospy.logerr('MissionCommanderGUI:_stop_vehicle_cb: %s',e)
				QMessageBox.warning(self._widget, 'Error', 'Error sending the command')
	
				
	def _stop_teleop_cb(self):
		'''
			Sends the stop teleop command IGC
		'''
		msg = Bool()	
		msg.data = True
		
		ret = QMessageBox.question(self._widget, "Stop Arm Teleop", 'Do you want to stop the arm teleoperation?', QMessageBox.Ok, QMessageBox.Cancel)
		
		if ret == QMessageBox.Ok:
			self.teleop_done_pub.publish(msg)
	
				
				
	def _set_control_mode_cb(self):
		'''
			Sets the platform control mode
		'''
		msg = SetControlMode()	
		msg.mode = self._widget.comboBox_control_mode.currentText()
		
		ret = QMessageBox.question(self._widget, "Set Control Mode", 'Set control mode to %s?'%msg.mode, QMessageBox.Ok, QMessageBox.Cancel)
		
		if ret == QMessageBox.Ok:
		
			try:
				self._set_control_mode_service_client(msg.mode) 
			except rospy.ROSInterruptException,e: 
				rospy.logerr('MissionCommanderGUI:_set_control_mode_cb: %s',e)
				QMessageBox.warning(self._widget, 'Error', 'Error sending the command')
			except rospy.ServiceException,e: 
				rospy.logerr('MissionCommanderGUI:_set_control_mode_cb: %s',e)
				QMessageBox.warning(self._widget, 'Error', 'Error sending the command')
				
	
	def _initialize_platform_cb(self):
		'''
			Initialize platform encoders
		'''
		msg = Empty()
		
		
		ret = QMessageBox.question(self._widget, "Init platform", 'Are you sure of initializing?', QMessageBox.Ok, QMessageBox.Cancel)
		
		if ret == QMessageBox.Ok:
			try:
				self._initialize_platform_service_client()
			except rospy.ROSInterruptException,e: 
				rospy.logerr('MissionCommanderGUI:_initialize_platform_cb: %s',e)
				QMessageBox.warning(self._widget, 'Error', 'Error sending the command')
			except rospy.ServiceException,e: 
				rospy.logerr('MissionCommanderGUI:_initialize_platform_cb: %s',e)
				QMessageBox.warning(self._widget, 'Error', 'Error sending the command')
	
	
	def _reset_steering_encoder_cb(self):
		'''
			Resets the steering encoder
		'''
		msg = Empty()
		ret = QMessageBox.question(self._widget, "Reset Encoder", 'Are you sure of resetting the encoder?', QMessageBox.Ok, QMessageBox.Cancel)
		
		if ret == QMessageBox.Ok:
			try:
				self._reset_steering_encoder_service_client()
			except rospy.ROSInterruptException,e: 
				rospy.logerr('MissionCommanderGUI:_reset_steering_encoder_cb: %s',e)
				QMessageBox.warning(self._widget, 'Error', 'Error sending the command')
			except rospy.ServiceException,e: 
				rospy.logerr('MissionCommanderGUI:_reset_steering_encoder_cb: %s',e)
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
		
		
	def _localization_state_cb(self, msg):
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
	
	def _pan_state_cb(self, msg):
		'''
			Callback for DynamixelState msg
			@param msg: received message
			@type msg: dynamixel_msgs/JointState
		'''
		self._components_state['pan_state']['state'] = msg
		self._components_state['pan_state']['time'] = rospy.Time.now()
		
	def _tilt_state_cb(self, msg):
		'''
			Callback for DynamixelState msg
			@param msg: received message
			@type msg: dynamixel_msgs/JointState
		'''
		self._components_state['tilt_state']['state'] = msg
		self._components_state['tilt_state']['time'] = rospy.Time.now()
	
	def _pad_state_cb(self, msg):
		'''
			Callback for PadStatus msg
			@param msg: received message
			@type msg: robospect_msgs/PadStatus
		'''
		self._components_state['pad_state']['state'] = msg
		self._components_state['pad_state']['time'] = rospy.Time.now()
	
	def _contact_switches_state_cb(self, msg):
		'''
			Callback for Switches msg
			@param msg: received message
			@type msg: geometry_msgs/Quaternion
		'''
		self._components_state['contact_switches_state']['state'] = msg
		self._components_state['contact_switches_state']['time'] = rospy.Time.now()
	
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
		except RuntimeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_current_waypoint.setText('%s'%self._components_state['mission_state']['state'].current_point)
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		except RuntimeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		
		
		# Mission waypoints
		try:
			self._widget.lineEdit_waypoints.setText('%d'%self.waypoints_markers.getSizeMissionPoints())
		except AttributeError,e:
			pass
			#rospy.logerr('MissionCommanderGUI: %s'%e)
		
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
		
		
		# Arm
		try:
			self._widget.lineEdit_arm_state.setText(self._components_state['mission_state']['state'].arm_state)	# State
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_arm_x.setText('%.3f'%self._components_state['mission_state']['state'].tip_x)	# arm tip X
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_arm_y.setText('%.3f'%self._components_state['mission_state']['state'].tip_y)	# arm tip Y
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_arm_z.setText('%.3f'%self._components_state['mission_state']['state'].tip_z)	# arm tip Z
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_arm_q1.setText('%.3f'%self._components_state['mission_state']['state'].tip_q1)	# arm tip q1
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_arm_q2.setText('%.3f'%self._components_state['mission_state']['state'].tip_q2)	# arm tip q2
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_arm_q3.setText('%.3f'%self._components_state['mission_state']['state'].tip_q3)	# arm tip q3
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		
		
		if len(self._components_state['mission_state']['state'].arm_joints) == 7:
			
			try:
				self._widget.lineEdit_arm_j1.setText('%.3f'%self._components_state['mission_state']['state'].arm_joints[0])	#j1 arm
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
			try:
				self._widget.lineEdit_arm_j2.setText('%.3f'%self._components_state['mission_state']['state'].arm_joints[1])	#j2 arm
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
			try:
				self._widget.lineEdit_arm_j3.setText('%.3f'%self._components_state['mission_state']['state'].arm_joints[2])	#j3 arm
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
			try:
				self._widget.lineEdit_arm_j4.setText('%.3f'%self._components_state['mission_state']['state'].arm_joints[3])	#j4 arm
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
			try:
				self._widget.lineEdit_arm_j5.setText('%.3f'%self._components_state['mission_state']['state'].arm_joints[4])	#j5 arm
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
			try:
				self._widget.lineEdit_arm_j6.setText('%.3f'%self._components_state['mission_state']['state'].arm_joints[5])	#j6 arm
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
			try:
				self._widget.lineEdit_arm_j7.setText('%.3f'%self._components_state['mission_state']['state'].arm_joints[6])	#j7 arm
			except AttributeError,e:
				rospy.logerr('MissionCommanderGUI: %s'%e)
			
		
		# Cameras
		try:
			self._widget.lineEdit_camera_state.setText(self._components_state['mission_state']['state'].camera_state)	# State
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_image_name_1.setText(self._components_state['mission_state']['state'].image_file_name1)	# Image file 1
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_image_name_2.setText(self._components_state['mission_state']['state'].image_file_name2)	# Image file 2
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_stereo_name_1.setText(self._components_state['mission_state']['state'].stereo_file_name1)	# Stereo file 1
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_stereo_name_2.setText(self._components_state['mission_state']['state'].stereo_file_name2)	# Stereo file 2
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_profile_name.setText(self._components_state['mission_state']['state'].profile_file_name)	# Profile name
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		
		try:
			self._widget.lineEdit_crack_pixel_x.setText('%d'%self._components_state['mission_state']['state'].crack_pixel_x)	#crack pixel x
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_crack_pixel_y.setText('%d'%self._components_state['mission_state']['state'].crack_pixel_y)	#crack pixel y
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_crack_pos_x.setText('%.3lf'%self._components_state['mission_state']['state'].crack_position_x)	#crack position x
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_crack_pos_y.setText('%.3lf'%self._components_state['mission_state']['state'].crack_position_y)	#crack position y
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_crack_pos_z.setText('%.3lf'%self._components_state['mission_state']['state'].crack_position_z)	#crack position z
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
			
		
		# Ultrasounds
		try:
			self._widget.lineEdit_ultrasounds_state.setText(self._components_state['mission_state']['state'].ultrasonic_state)	# State
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_crack_width.setText('%.3lf'%self._components_state['mission_state']['state'].crack_width)	#crack width
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_crack_depth.setText('%.3lf'%self._components_state['mission_state']['state'].crack_depth)	#crack depth
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
			
		
		# Pan-Tilt
		try:
			self._widget.lineEdit_pan.setText('%.3lf'%self._components_state['mission_state']['state'].vehicle_state.pan_angle)	
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_tilt.setText('%.3lf'%self._components_state['mission_state']['state'].vehicle_state.tilt_angle)	
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
				
		# Vehicle controller
		try:
			self._widget.lineEdit_controller_state.setText(self._components_state['platform_controller_state']['state'].state_description)	# State
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_controller_mode.setText(self._components_state['platform_controller_state']['state'].control_mode)	# mode
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		
		## Nav200
		try:
			self._widget.lineEdit_nav200_state.setText(self._components_state['localization_state']['state'].state)	# State
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		try:
			self._widget.lineEdit_nav200_x.setText('%.3lf'%self._components_state['localization_state']['state'].x)	# x
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_nav200_y.setText('%.3lf'%self._components_state['localization_state']['state'].y)	# x
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_nav200_theta.setText('%.3lf'%self._components_state['localization_state']['state'].theta)	# x
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_nav200_beacons.setText('%d'%self._components_state['localization_state']['state'].refnumb)	# beacons
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			if self._components_state['localization_state']['state'].quality < 0:
				self._widget.progressBar_nav200_quality.setValue(0)
			else:	
				self._widget.progressBar_nav200_quality.setValue(int(self._components_state['localization_state']['state'].quality))
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
		
		## PAD
		try:
			self._widget.lineEdit_pad_speed_level.setText('%.3lf'%self._components_state['pad_state']['state'].vehicle_speed_level)	# speed level
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_pad_steering_pos.setText('%.3lf'%self._components_state['pad_state']['state'].desired_angular_position)	# steering pos
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_pad_speed_lvl.setText('%.3lf'%self._components_state['pad_state']['state'].desired_linear_speed)	# linear speed
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_pad_crane_joint.setText('%s'%self._components_state['pad_state']['state'].current_joint)	# current joint
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_controller_mode_2.setText('%s'%self._components_state['pad_state']['state'].platform_mode)	# current platform mode
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		
		try:
			self._widget.lineEdit_pad_crane_speed_lvl.setText('%.3lf'%self._components_state['pad_state']['state'].arm_speed_level)	# arm speed lvl
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_pad_joint_speed.setText('%.3lf'%self._components_state['pad_state']['state'].current_joint_speed)	# joint speed
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		
		## PAN STATE
		try:
			self._widget.lineEdit_pan_motor_temp.setText('%.3lf'%self._components_state['pan_state']['state'].motor_temps[0])	# temp
		except AttributeError,e:
			pass
			#rospy.logerr('MissionCommanderGUI: %s'%e)	
		except IndexError,e:
			pass
			#rospy.logerr('MissionCommanderGUI: %s'%e)	
			
		try:
			self._widget.lineEdit_pan_motor_vel.setText('%.3lf'%self._components_state['pan_state']['state'].velocity)	# velocity
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_pan_motor_load.setText('%.3lf'%self._components_state['pan_state']['state'].load)	# load
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_pan_motor_error.setText('%.3lf'%self._components_state['pan_state']['state'].error)	# error
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		
		## TILT STATE
		try:
			self._widget.lineEdit_tilt_motor_temp.setText('%.3lf'%self._components_state['tilt_state']['state'].motor_temps[0])	# temp
		except AttributeError,e:
			pass
			#rospy.logerr('MissionCommanderGUI: %s'%e)	
		except IndexError,e:
			pass
			#rospy.logerr('MissionCommanderGUI: %s'%e)
			
		try:
			self._widget.lineEdit_tilt_motor_vel.setText('%.3lf'%self._components_state['tilt_state']['state'].velocity)	# velocity
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)
			
		
		try:
			self._widget.lineEdit_tilt_motor_load.setText('%.3lf'%self._components_state['tilt_state']['state'].load)	# load
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		try:
			self._widget.lineEdit_tilt_motor_error.setText('%.3lf'%self._components_state['tilt_state']['state'].error)	# error
		except AttributeError,e:
			rospy.logerr('MissionCommanderGUI: %s'%e)	
		
		## FLAGS status
		t_now = rospy.Time.now()
		for component in self._components_state:
			if self._components_state[component]['label']:
				if (t_now - self._components_state[component]['time']).to_sec() > TIMEOUT_TOPIC:
					self._components_state[component]['label'].setPixmap(self._pixmap_red)
				else:
					if component == 'localization_state':
						if self._components_state['localization_state']['state'].quality < NAV200_MIN_QUALITY:
							self._components_state[component]['label'].setPixmap(self._pixmap_orange)
						else:
							self._components_state[component]['label'].setPixmap(self._pixmap_green)
					else:
						self._components_state[component]['label'].setPixmap(self._pixmap_green)
						
						if component == 'pad_state':
							if self._components_state[component]['state'].deadman_active:
								self._components_state[component]['label_vehicle'].setPixmap(self._pixmap_green)
							else:
								self._components_state[component]['label_vehicle'].setPixmap(self._pixmap_red)
							
							if self._components_state[component]['state'].arm_deadman_active:
								self._components_state[component]['label_crane'].setPixmap(self._pixmap_green)
							else:
								self._components_state[component]['label_crane'].setPixmap(self._pixmap_red)
						
						if component == 'pan_state' or component == 'tilt_state':
							if self._components_state[component]['state'].is_moving:
								self._components_state[component]['label_moving'].setPixmap(self._pixmap_green)
							else:
								self._components_state[component]['label_moving'].setPixmap(self._pixmap_red)
						
						if component == 'contact_switches_state':
							if self._components_state[component]['state'].x == 0:
								self._components_state[component]['label_sw1'].setPixmap(self._pixmap_green)
							else:
								self._components_state[component]['label_sw1'].setPixmap(self._pixmap_red)
							
							if self._components_state[component]['state'].y == 0:
								self._components_state[component]['label_sw2'].setPixmap(self._pixmap_green)
							else:
								self._components_state[component]['label_sw2'].setPixmap(self._pixmap_red)
							
							if self._components_state[component]['state'].z == 0:
								self._components_state[component]['label_sw3'].setPixmap(self._pixmap_green)
							else:
								self._components_state[component]['label_sw3'].setPixmap(self._pixmap_red)
							
							if self._components_state[component]['state'].w == 0:
								self._components_state[component]['label_sw4'].setPixmap(self._pixmap_green)
							else:
								self._components_state[component]['label_sw4'].setPixmap(self._pixmap_red)
			
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
        
