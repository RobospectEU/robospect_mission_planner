#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy 

import time, threading, copy

from robotnik_msgs.msg import State
from std_msgs.msg import String
from robospect_msgs.msg import PlatformCommand, PlatformState, MissionState
from robospect_msgs.msg import State as RobospectState
from robospect_msgs.srv import PlatformCommandSrv
from robotnik_trajectory_planner.msg import CartesianEuler, JointByJoint
from robotnik_trajectory_planner.msg import State as TrajectoryPlannerState
from robotnik_trajectory_control.srv import TrajExecActions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from tf.transformations import euler_from_quaternion	
from tf import TransformListener, Exception as tfException, ConnectivityException, LookupException, ExtrapolationException

import actionlib
from actionlib_msgs.msg import GoalStatus
from robospect_planner.msg import goal, GoToGoal, GoToAction
from geometry_msgs.msg import Pose2D, PointStamped, Point


DEFAULT_FREQ = 100.0
MAX_FREQ = 500.0

# COMMANDS
COMMAND_ADVANCE = 'advance'
COMMAND_MOVE_CRANE = 'moveCrane'
COMMAND_FOLD_CRANE = 'foldCrane'
DONE_ADVANCE = 'doneAdvance'
DONE_MOVE_CRANE = 'doneMoveCrane'
DONE_FOLD_CRANE = 'doneFoldCrane'
FAIL_ADVANCE = 'failAdvance'
FAIL_MOVE_CRANE = 'failMoveCrane'
FAIL_FOLD_CRANE = 'failFoldCrane'

COMMAND_CANCEL = 'cancel'
# State of the command execution
COMMAND_STATE_INIT = 1
COMMAND_STATE_WAITING = 2
COMMAND_STATE_ENDED	= 3
# 
COMMAND_ADVANCE_SPEED = 0.4
COMMAND_ADVANCE_MAX_SPEED = 0.5
# offset applied to the crack position
X_DEFAULT_OFFSET = 0.0
Y_DEFAULT_OFFSET = -1.0
Z_DEFAULT_OFFSET = -0.4
# timeout in secs
DEFAULT_COMMAND_TIMEOUT = 120.0
# Commands to send to the Robotnik Trajectory Control Node
RT_TRAJ_EXE_SET_TIP_FIRST=0
RT_TRAJ_EXE_UNSET_TIP_FIRST=1

# Client based on ActionServer to send goals to the purepursuit node
class PurePursuitClient():
	
	def __init__(self, planner_name):
		self.planner_name = planner_name
		# Creates the SimpleActionClient, passing the type of the action
		# (GoTo) to the constructor.
		self.client = actionlib.SimpleActionClient(planner_name, GoToAction)

	## @brief Sends the goal to 
	## @return 0 if OK, -1 if no server, -2 if it's tracking a goal at the moment
	def goTo(self, goal_list):
		# Waits until the action server has started up and started
		# listening for goals.
		if self.client.wait_for_server(timeout = rospy.Duration(3.0) ):
			#if self.getState() != GoalStatus.LOST:
			#	rospy.loginfo('PurepursuitClient: planner is tracking a goal')
			#	return -2
				
			g = GoToGoal(target = goal_list)
			rospy.loginfo('PurepursuitClient: Sendig %d waypoints'%(len(goal_list)))
			self.client.send_goal(g)
			return 0
		else:
			rospy.logerr('PurepursuitClient: Error waiting for server')
			return -1
	
	## @brief cancel the current goal
	def cancel(self):		
		rospy.loginfo('PurepursuitClient: cancelling the goal')
		self.client.cancel_goal()
	
	## @brief Get the state information for this goal
    ##
    ## Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED,
    ## PREEMPTED, ABORTED, SUCCEEDED, LOST.
    ##
    ## @return The goal's state. Returns LOST if this
    ## SimpleActionClient isn't tracking a goal.
	def getState(self):
		return self.client.get_state()
		
	def getStateString(self):
		state = self.client.get_state()
		
		if state == GoalStatus.PENDING:
			return "PENDING"
		elif state == GoalStatus.SUCCEEDED:
			return "SUCCEEDED"
		elif state == GoalStatus.ACTIVE:
			return "ACTIVE"
		elif state == GoalStatus.PREEMPTED:
			return "PREEMPTED"
		elif state == GoalStatus.ABORTED:
			return "ABORTED"
		elif state == GoalStatus.REJECTED:
			return "REJECTED"
		elif state == GoalStatus.LOST:
			return "LOST"
		else:
			return "UNKNOWN"
		
	## @brief Returns ret if OK, otherwise -1
	def getResult(self):
		ret = self.client.get_result()
		if not ret:
			return -1
		
		else:
			return ret
			
			
# Client based on topics to send the Cartesian-Euler trajectory commands
class CraneTrajectoryClient():
	
	
	def __init__(self, planner_cartesian_command_topic, planner_joint_command_topic, planner_state_topic):
		
		self._planner_cartesian_command_topic = planner_cartesian_command_topic
		self._planner_joint_command_topic = planner_joint_command_topic
		self._planner_state_topic = planner_state_topic
		
		self._cartesian_command_pub = rospy.Publisher(self._planner_cartesian_command_topic, CartesianEuler, queue_size = 10)
		self._joint_command_pub = rospy.Publisher(self._planner_joint_command_topic, JointByJoint, queue_size = 10)
		self._state_sub = rospy.Subscriber(self._planner_state_topic, TrajectoryPlannerState, self._state_cb, queue_size = 10)
	
		self._state = TrajectoryPlannerState()
		self._state_topic_timer = rospy.Time.now()
		self._state_topic_timeout = 1.0
		
	def _state_cb(self, msg):
		'''
			Callback for TrajectoryPlannerState msg
			@param msg: received message
			@type msg: robotnik_trajectory_planner/State
		'''
		self._state_topic_timer = rospy.Time.now()
		self._state = msg	
	
	def goTo(self, goal):
		'''
			@brief Sends the command to the component
			@param goal as CartesianEuler or JointByJoint
			@return 0 if OK, -1 if no server, -2 if it's tracking a goal at the moment
		'''
		if isinstance(goal, CartesianEuler):
			self._cartesian_command_pub.publish(goal)
			return 0
		elif isinstance(goal, JointByJoint):
			self._joint_command_pub.publish(goal)
			return 0
		return -1
		
	def cancel(self):		
		'''
			@brief cancel the current goal
		'''
		pass
	
	def getState(self):	
		return self._state
		
		
	def getStateString(self):
		
		return self._state.state.state_description	
	
	def getResult(self):
		'''
			@brief Returns ret if OK, otherwise -1
		'''
		
		return 0
		
	def foldCrane(self):
		'''
			@brief sends the command to move the crane to home position
		'''
		msg = JointByJoint()
		msg.joints = ['crane_first_joint', 'crane_second_joint', 'crane_third_joint', 'crane_fourth_joint', 'crane_sixth_joint', 'crane_tip_joint']
		msg.values = [0.0, 0.1, 0.04, 0.2, -1.0, 0.0]
		msg.relative = False
		
		return self.goTo(msg)



	
# 
class RobospectPlatformMissionManager:
	
	def __init__(self, args):
		
		self.node_name = rospy.get_name().replace('/','')
		self.desired_freq = args['desired_freq'] 
		# Checks value of freq
		if self.desired_freq <= 0.0 or self.desired_freq > MAX_FREQ:
			rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name,self.desired_freq, DEFAULT_FREQ))
			self.desired_freq = DEFAULT_FREQ
	
		self._odom_frame_id = args['odom_frame_id']
		self._map_frame_id = args['map_frame_id']
		self._base_frame_id = args['base_frame_id']
		self._jointstate_topic = args['joint_states_topic']
		self._odom_topic = args['odom_topic']
		self._robot_state_topic = args['robot_state_topic']
		self._crane_joints = args['crane_joints']
		self._crane_tip_frame_id = args['crane_tip_frame_id']
		self._arm_frame_id = args['arm_frame_id']
		self._camera_frame_id = args['camera_frame_id']
		self._joint_linear_speed_name = args['joint_linear_speed']
		self._publish_mission_state = args['publish_mission_state']
		self._base_planner_name = args['base_planner_name']
		self._command_advance_speed = abs(args['advance_speed'])
		self._command_advance_max_speed = abs(args['advance_max_speed'])
		self._relative_navigation = args['relative_navigation']
		self._trajectory_planner_cartesian_command_topic = args['trajectory_planner_cartesian_command_topic']
		self._trajectory_planner_joint_command_topic = args['trajectory_planner_joint_command_topic']
		self._trajectory_planner_state_topic = args['trajectory_planner_state_topic']
		self._point_offset = Point()
		self._point_offset.x = X_DEFAULT_OFFSET
		self._point_offset.y = Y_DEFAULT_OFFSET
		self._point_offset.z = Z_DEFAULT_OFFSET
		self._crack_frame_id = args['crack_frame_id']
		self._crack_approach_arm_frame_id = args['crack_approach_arm_frame_id']
		self._crack_approach_crane_frame_id = args['crack_approach_crane_frame_id']
		self._traj_exec_actions_name = args['traj_exec_actions_name']
		self._avoid_crane_movement = args['avoid_crane_movement']
		
		if self._command_advance_speed > self._command_advance_max_speed:
			self._command_advance_speed = self._command_advance_max_speed
		
		self.real_freq = 0.0
		
		# Saves the state of the component
		self.state = State.INIT_STATE
		# Saves the previous state
		self.previous_state = State.INIT_STATE
		# flag to control the initialization of the component
		self.initialized = False
		# flag to control the initialization of ROS stuff
		self.ros_initialized = False
		# flag to control that the control loop is running
		self.running = False
		# Variable used to control the loop frequency
		self.time_sleep = 1.0 / self.desired_freq
		# State msg to publish
		self.msg_state = State()
		# Timer to publish state
		self.publish_state_timer = 1
		
		self._platform_state = PlatformState()
		# Saves the last platform command (and time) received
		self._platform_command_time = rospy.Time.now()
		self._platform_commands = []
		# Current command being executed
		self._platform_current_command = PlatformCommand()
		# Current robot odometry
		self._odometry = Odometry()
		self._odometry_time = rospy.Time.now()
		# Current robot Joint states
		self._joint_state = JointState()
		self._joint_state_time = rospy.Time.now()
		# Current robot state
		self._robot_state = State()
		self._robot_state_time = rospy.Time.now()
		# Current linear speed
		self._linear_speed = 0.0
		# Saves the state of the command execution
		self.command_state = COMMAND_STATE_INIT
		# Flag active under requirement to cancel the current command
		self._cancel_command = False
		
		
		self._joints_dict = {}
		# Inits joint states
		for i in self._crane_joints:
			# Only saves the position
			self._joints_dict[i] = 0.0
		
		self._transform_listener = TransformListener()
		
		self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
		# Timeout applied when executing an action
		self._command_timeout = DEFAULT_COMMAND_TIMEOUT
		# Saves the time when an action has been sent to every component
		self._command_init_time = None
		
			
	def setup(self):
		'''
			Initializes de hand
			@return: True if OK, False otherwise
		'''
		self.initialized = True
		
		return 0
		
		
	def rosSetup(self):
		'''
			Creates and inits ROS components
		'''
		if self.ros_initialized:
			return 0
		
		# Publishers
		self._state_publisher = rospy.Publisher('~state', State, queue_size=10)
		self._platform_state_pub = rospy.Publisher('/platform_state', PlatformState, queue_size=10)
		if self._publish_mission_state:
			self._mission_state_pub = rospy.Publisher('/mission_state', MissionState, queue_size=10)
			self._mission_state = MissionState()
			self._mission_state.vehicle_state = self._platform_state
			self._mission_state.mission_state = 'IDLE'
		#publishes the crack to reach
		self._crack_publisher = rospy.Publisher('~crack', PointStamped, queue_size=10)
		self._crack_approach_arm_publisher = rospy.Publisher('~crack_approach_arm', PointStamped, queue_size=10)
		self._crack_approach_crane_publisher = rospy.Publisher('~crack_approach_crane', PointStamped, queue_size=10)
		
		self._crack_point = PointStamped()
		self._crack_approach_arm_point = PointStamped()
		self._crack_approach_crane_point = PointStamped()
		self._crack_point.header.frame_id = self._crack_frame_id
		self._crack_approach_arm_point.header.frame_id = self._crack_approach_arm_frame_id
		self._crack_approach_crane_point.header.frame_id = self._crack_approach_crane_frame_id
		
		
		# Subscribers
		# topic_name, msg type, callback, queue_size
		#self._platform_command_sub = rospy.Subscriber('/platform_command', PlatformCommand, self._platform_command_cb, queue_size = 10)
		self._odom_sub = rospy.Subscriber(self._odom_topic, Odometry, self._odom_cb, queue_size = 10)
		self._joint_state_sub = rospy.Subscriber(self._jointstate_topic, JointState, self._joint_state_cb, queue_size = 10)
		self._robot_state_sub = rospy.Subscriber(self._robot_state_topic, RobospectState, self._robot_state_cb, queue_size = 10)
		
		# Service Servers
		self._platform_command_server = rospy.Service('/platform_command', PlatformCommandSrv, self._platform_command_cb)
		
		# Service Clients
		# Service to set the order of joint movements of the crane
		self._rt_traj_exe_actions_service_client = rospy.ServiceProxy(self._traj_exec_actions_name, TrajExecActions)
		#rospy.loginfo('rosSetup: Connecting to service %s'%self._traj_exec_actions_name )
		
		# ret = self.service_client.call(ServiceMsg)
		# Simple Action Client
		self._base_move_client = PurePursuitClient(self._base_planner_name)
		
		# interface to communicate with the Crane
		self._crane_move_client = CraneTrajectoryClient(planner_cartesian_command_topic = self._trajectory_planner_cartesian_command_topic, 
			planner_joint_command_topic = self._trajectory_planner_joint_command_topic, planner_state_topic = self._trajectory_planner_state_topic)
		
		
		self.ros_initialized = True
		
		self.publishROSstate()
		
		return 0
		
		
	def shutdown(self):
		'''
			Shutdowns device
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.initialized:
			return -1
		rospy.loginfo('%s::shutdown'%self.node_name)
		
		# Cancels current timers
		self.t_publish_state.cancel()
		
		self._state_publisher.unregister()
		
		self.initialized = False
		
		return 0
	
	
	def rosShutdown(self):
		'''
			Shutdows all ROS components
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.ros_initialized:
			return -1
		
		# Performs ROS topics & services shutdown
		self._state_publisher.unregister()
		
		self.ros_initialized = False
		
		return 0
			
	
	def stop(self):
		'''
			Creates and inits ROS components
		'''
		self.running = False
		
		return 0
	
	
	def start(self):
		'''
			Runs ROS configuration and the main control loop
			@return: 0 if OK
		'''
		self.rosSetup()
		
		if self.running:
			return 0
			
		self.running = True
		
		self.controlLoop()
		
		return 0
	
	
	def controlLoop(self):
		'''
			Main loop of the component
			Manages actions by state
		'''
		
		while self.running and not rospy.is_shutdown():
			t1 = time.time()
			
			if self.state == State.INIT_STATE:
				self.initState()
				
			elif self.state == State.STANDBY_STATE:
				self.standbyState()
				
			elif self.state == State.READY_STATE:
				self.readyState()
				
			elif self.state == State.EMERGENCY_STATE:
				self.emergencyState()
				
			elif self.state == State.FAILURE_STATE:
				self.failureState()
				
			elif self.state == State.SHUTDOWN_STATE:
				self.shutdownState()
				
			self.allState()
			
			t2 = time.time()
			tdiff = (t2 - t1)
			
			
			t_sleep = self.time_sleep - tdiff
			
			if t_sleep > 0.0:
				try:
					rospy.sleep(t_sleep)
				except rospy.exceptions.ROSInterruptException:
					rospy.loginfo('%s::controlLoop: ROS interrupt exception'%self.node_name)
					self.running = False
			
			t3= time.time()
			self.real_freq = 1.0/(t3 - t1)
		
		self.running = False
		# Performs component shutdown
		self.shutdownState()
		# Performs ROS shutdown
		self.rosShutdown()
		rospy.loginfo('%s::controlLoop: exit control loop'%self.node_name)
		
		return 0
		
	def _update_platform_state(self):
		'''
			Gathers and updates the platform data from different components
		'''
		
		# Publish the current state of the platform
		position = None
		quaternion = None
		
		self._platform_state.crane_joints = [self._joints_dict[self._crane_joints[0]], self._joints_dict[self._crane_joints[1]], self._joints_dict[self._crane_joints[2]], self._joints_dict[self._crane_joints[3]], self._joints_dict[self._crane_joints[4]],
		self._joints_dict[self._crane_joints[5]], self._joints_dict[self._crane_joints[6]]]
		# Updates transform from crane_tip to map
		if self._transform_listener.frameExists(self._crane_tip_frame_id) and self._transform_listener.frameExists(self._map_frame_id):
			
			try:
				t = self._transform_listener.getLatestCommonTime(self._map_frame_id, self._crane_tip_frame_id)
				position, quaternion = self._transform_listener.lookupTransform(self._map_frame_id, self._crane_tip_frame_id, t)
				self._platform_state.crane_x = position[0]
				self._platform_state.crane_y = position[1]
				self._platform_state.crane_z = position[2]
				self._platform_state.crane_q1 = quaternion[0]
				self._platform_state.crane_q2 = quaternion[1]
				self._platform_state.crane_q3 = quaternion[2]
				self._platform_state.crane_q4 = quaternion[3]
				
				#print position, quaternion
			except tfException, e:
				rospy.logerr('%s::_update_platform_state: %s'%(self.node_name, e))
			#Another exception when there is no transform
			except ConnectivityException, e:
				rospy.logerr('%s::_update_platform_state: %s'%(self.node_name, e))
			except LookupException, e:
				rospy.logerr('%s::_update_platform_state: %s'%(self.node_name, e))
			except ExtrapolationException, e:
				rospy.logerr('%s::_update_platform_state: %s'%(self.node_name, e))
		#else:
		#	rospy.logerr('%s::rosPublish: No transform between %s -> %s'%(self.node_name, self._crane_tip_frame_id, self._map_frame_id))
		
		# Vehicle position & speed
		if self._transform_listener.frameExists(self._base_frame_id) and self._transform_listener.frameExists(self._map_frame_id):
			try:
				t = self._transform_listener.getLatestCommonTime(self._map_frame_id, self._base_frame_id)
				position, quaternion = self._transform_listener.lookupTransform(self._map_frame_id, self._base_frame_id, t)
				self._platform_state.vehicle_x = position[0]
				self._platform_state.vehicle_y = position[1]
				(roll, pitch, yaw) = euler_from_quaternion(quaternion)
				self._platform_state.vehicle_theta = yaw
				
				#print position, quaternion
			except tfException, e:
				rospy.logerr('%s::_update_platform_state: %s'%(self.node_name, e))
			#Another exception when there is no transform
			except ConnectivityException, e:
				rospy.logerr('%s::_update_platform_state: %s'%(self.node_name, e))
			except LookupException, e:
				rospy.logerr('%s::_update_platform_state: %s'%(self.node_name, e))
			except ExtrapolationException, e:
				rospy.logerr('%s::_update_platform_state: %s'%(self.node_name, e))
		
		self._platform_state.vehicle_linear_speed = self._linear_speed
		self._platform_state.vehicle_angular_speed = self._odometry.twist.twist.angular.z
		
		# state
		self._platform_state.state = self.stateToString(self.state)
		# Current command
		self._platform_state.command = self._command_to_string(self._platform_current_command)
		# battery (fake)
		self._platform_state.battery_level = 75.0
	
	
	def _transform_point_to_frame(self, point, frame_id, offset = Point()):
		'''
			Transforms a point into the desired frame
			@param point as geometry_msg/PointStamped, point to transform
			@param point as geometry_msg/Point, offset applied to the point before transforming
			@param frame_id as string
			@return 0, PointStamped if OK or -1,PointStamped otherwise
		'''
		if not self._transform_listener.frameExists(frame_id):
			rospy.logerr('%s:_transform_point_to_frame: frame %s does not exist',self.node_name, frame_id)
			return -1, point
			
		if not self._transform_listener.frameExists(point.header.frame_id):
			rospy.logerr('%s:_transform_point_to_frame: frame %s does not exist',self.node_name, point.header.frame_id)
			return -1, point
			
		point.header.stamp = rospy.Time.now() - rospy.Time(1)
		#point.header.stamp = rospy.Time.now() - rospy.Time.from_sec(0.5)
		
		try:
			
			point.point.x += (offset.x)
			point.point.y += (offset.y)
			point.point.z += (offset.z)
			#print 'point after offset'
			#print point
			new_point = self._transform_listener.transformPoint(frame_id, point)
			rospy.loginfo('%s:_transform_point_to_frame: point transformed from frame %s to frame %s', self.node_name,point.header.frame_id, frame_id)
			#print new_point
			
			return 0, new_point
			
		except tfException, e:
			rospy.logerr('%s::_transform_point_to_frame: %s'%(self.node_name, e))
		#Another exception when there is no transform
		except ConnectivityException, e:
			rospy.logerr('%s::_transform_point_to_frame: %s'%(self.node_name, e))
		except LookupException, e:
			rospy.logerr('%s::_transform_point_to_frame: %s'%(self.node_name, e))
		except ExtrapolationException, e:
			rospy.logerr('%s::_transform_point_to_frame: %s'%(self.node_name, e))
		
		return -1, point	
	
		
	def rosPublish(self):
		'''
			Publish topics at standard frequency
		'''
		
				
		self._platform_state_pub.publish(self._platform_state)
		
		
		# Publish mission state for testing the GCS
		if self._publish_mission_state:
			self._mission_state_pub.publish(self._mission_state)
		
		t_now = rospy.Time.now()
		
		self._crack_point.header.stamp = t_now
		self._crack_approach_arm_point.header.stamp = t_now
		self._crack_approach_crane_point.header.stamp = t_now
		
		self._crack_publisher.publish(self._crack_point)
		self._crack_approach_arm_publisher.publish(self._crack_approach_arm_point)
		self._crack_approach_crane_publisher.publish(self._crack_approach_crane_point)
					
		return 0
		
	
	def initState(self):
		'''
			Actions performed in init state
		'''
		
		if not self.initialized:
			self.setup()
			
		else: 		
			self.switchToState(State.STANDBY_STATE)
		
		
		return
	
	
	def standbyState(self):
		'''
			Actions performed in standby state
		'''
		##self.switchToState(State.READY_STATE)
		# Checks for new commands
		if len(self._platform_commands) > 0:
			self._platform_current_command = self._platform_commands[0]
			self._platform_commands.remove(self._platform_current_command)
			rospy.loginfo('%s:standbyState: New command %s. going to READY', self.node_name, self._platform_current_command.command )
			self.switchToState(State.READY_STATE)
		
		return
	
	
	def readyState(self):
		'''
			Actions performed in ready state
		'''
		# Send Command to the robot
		if self.command_state == COMMAND_STATE_INIT:
			# Sends the command to the platform
			# ADVANCE
			if self._platform_current_command.command == COMMAND_ADVANCE:
				goal_list = []
				
				if self._relative_navigation:
					goal_list.append(goal(pose = Pose2D(self._platform_state.vehicle_x + self._platform_current_command.variables[0], self._platform_state.vehicle_y, 0.0), speed = self._command_advance_speed )) 
				else:					
					goal_list.append(goal(pose = Pose2D(self._platform_current_command.variables[0], 0.0, 0.0), speed = self._command_advance_speed )) 

				if self._base_move_client.goTo(goal_list) == 0:
					self._command_init_time = rospy.Time.now()
					self.command_state = COMMAND_STATE_WAITING
				else:
					rospy.logerr('%s::readyState: Error sending command to the platfom', self.node_name)
					self.command_state = COMMAND_STATE_ENDED
			
			# MOVE CRANE
			elif self._platform_current_command.command == COMMAND_MOVE_CRANE:
				traj_planner_state = self._crane_move_client.getState()
				
				if traj_planner_state.state.state == State.STANDBY_STATE and traj_planner_state.goal_state == 'IDLE':
					# Setting the order of joints movement
					action = TrajExecActions()
					action.action = RT_TRAJ_EXE_SET_TIP_FIRST
					ret = self._rt_traj_exe_actions_service_client.call(action.action)
					
					if not ret:
						rospy.logerr('%s::readyState: error communicating with rt_traj_exec', self.node_name)
						return
						
					rospy.loginfo('%s::readyState: New trajectory command %s to (%lf, %lf, %lf)', self.node_name, self._platform_current_command.command, self._platform_current_command.variables[0], self._platform_current_command.variables[1], self._platform_current_command.variables[2])
					# Tries to transform the crack point from arm_link to tip_link
					crack_point = PointStamped()
					crack_approach_arm_point = PointStamped()
					crack_approach_crane_point = PointStamped()
					
					crack_point.point.x = self._platform_current_command.variables[0]
					crack_point.point.y = self._platform_current_command.variables[1]
					crack_point.point.z = self._platform_current_command.variables[2]
					crack_point.header.frame_id = self._camera_frame_id
					
					
					# Receives coordinates based on camera frame
					# Transform the point into arm_base_link
					ret, crack_approach_arm_point = self._transform_point_to_frame(point = copy.deepcopy(crack_point), frame_id = self._arm_frame_id)
					
					if ret:
						rospy.logerr('%s::readyState: error transforming crack point', self.node_name)
						return
					
					# Applies offset to enable the arm operation close to the crack
					crack_approach_arm_point.point.x += self._point_offset.x
					crack_approach_arm_point.point.y += self._point_offset.y
					crack_approach_arm_point.point.z += self._point_offset.z
					
					
					try:
						t = self._transform_listener.getLatestCommonTime(self._arm_frame_id, self._crane_tip_frame_id)
						position, quaternion = self._transform_listener.lookupTransform(self._arm_frame_id, self._crane_tip_frame_id, t)
						
						# Applies the transformation between arm_base_link and crane_tip_link
						crack_approach_crane_point.point.x = crack_approach_arm_point.point.x + position[0]
						crack_approach_crane_point.point.y = crack_approach_arm_point.point.y + position[1]
						crack_approach_crane_point.point.z = crack_approach_arm_point.point.z + position[2]
						crack_approach_crane_point.header.frame_id = self._arm_frame_id
							
					except (tfException, ConnectivityException, LookupException, ExtrapolationException) as e:
						rospy.logerr('%s::readyState: %s'%(self.node_name, e))
						return 
					
					
					# transform the point into crane coordinates, ready to send to the controller
					ret, crack_approach_crane_point = self._transform_point_to_frame(point = copy.deepcopy(crack_approach_crane_point), frame_id = self._crane_tip_frame_id)
					
					if ret:
						rospy.logerr('%s::readyState: error transforming crack point', self.node_name)
						return
					
					# Publish the points related to base frame
					ret, self._crack_point = self._transform_point_to_frame(point = copy.deepcopy(crack_point), frame_id = self._base_frame_id) 
					ret, self._crack_approach_arm_point = self._transform_point_to_frame(point = copy.deepcopy(crack_approach_arm_point), frame_id = self._base_frame_id)  
					ret, self._crack_approach_crane_point = self._transform_point_to_frame(point = copy.deepcopy(crack_approach_crane_point), frame_id = self._base_frame_id)  
					
					#print 'Move the crane to (%lf, %lf, %lf)'%(crack_approach_crane_point.point.x, crack_approach_crane_point.point.y, crack_approach_crane_point.point.z) 
					
					# Sends the command to the crane 
					msg = CartesianEuler()
					msg.x = crack_approach_crane_point.point.x
					msg.y = crack_approach_crane_point.point.y
					msg.z = crack_approach_crane_point.point.z
					msg.roll = msg.pitch = msg.yaw = 0.0
					print 'sending msg: %s'%msg
					if self._avoid_crane_movement:
						self.command_state = COMMAND_STATE_ENDED
						rospy.loginfo('%s::readyState: command %s finished'%(self.node_name,self._platform_current_command.command))
						self._platform_current_command.command = DONE_MOVE_CRANE
					else:
						
						self._crane_move_client.goTo(msg)
						self._command_init_time = rospy.Time.now()
						# Give some time to activate the service
						rospy.sleep(2)
						self.command_state = COMMAND_STATE_WAITING
				else:
					self.command_state = COMMAND_STATE_ENDED
					rospy.loginfo('%s::readyState: Trajectory planner not ready for a new command (%s-%s)', self.node_name, traj_planner_state.state.state_description,traj_planner_state.goal_state)
					self._platform_current_command.command = FAIL_MOVE_CRANE
			
			
			# FOLD CRANE		
			elif self._platform_current_command.command == COMMAND_FOLD_CRANE:
				traj_planner_state = self._crane_move_client.getState()
				
				# TODO
				if traj_planner_state.state.state == State.STANDBY_STATE and traj_planner_state.goal_state == 'IDLE':
					# Setting the order of joints movement
					action = TrajExecActions()
					action.action = RT_TRAJ_EXE_UNSET_TIP_FIRST
					ret = self._rt_traj_exe_actions_service_client.call(action.action)
					if not ret:
						rospy.logerr('%s::readyState: error communicating with rt_traj_exec', self.node_name)
						return
						
					self._crane_move_client.foldCrane()
					self._command_init_time = rospy.Time.now()
					rospy.loginfo('%s::readyState: New trajectory command %s', self.node_name, self._platform_current_command.command)
					rospy.sleep(1)
					self.command_state = COMMAND_STATE_WAITING
				else:
					rospy.loginfo('%s::readyState: Trajectory planner not ready for a new command (%s-%s)', self.node_name, traj_planner_state.state.state_description,traj_planner_state.goal_state)
			
			else:
				self.command_state = COMMAND_STATE_ENDED
		
		# Wait for the end
		elif self.command_state == COMMAND_STATE_WAITING:
			
			if self._platform_current_command.command == COMMAND_ADVANCE:
				action_state = self._base_move_client.getState()
				
				if action_state == GoalStatus.SUCCEEDED or action_state == GoalStatus.PREEMPTED or action_state == GoalStatus.ABORTED or action_state == GoalStatus.REJECTED or action_state == GoalStatus.LOST:	
					rospy.loginfo('%s::readyState: command %s finished'%(self.node_name,self._platform_current_command.command))
					self.command_state = COMMAND_STATE_ENDED
					self._platform_current_command.command = DONE_ADVANCE
				# Cancel requested
				elif self._cancel_command:
					rospy.loginfo('%s::readyState: cancelling command %s'%(self.node_name,self._platform_current_command.command))
					self._base_move_client.cancel()
					self.command_state = COMMAND_STATE_ENDED
					self._platform_current_command.command = FAIL_ADVANCE
					
				
			# MOVE CRANE
			elif self._platform_current_command.command == COMMAND_MOVE_CRANE:
				
				traj_planner_state = self._crane_move_client.getState()
				
				if traj_planner_state.state.state == State.STANDBY_STATE and traj_planner_state.goal_state == 'IDLE':
					self.command_state = COMMAND_STATE_ENDED
					rospy.loginfo('%s::readyState: command %s finished'%(self.node_name,self._platform_current_command.command))
					self._platform_current_command.command = DONE_MOVE_CRANE
				else:
					t_diff = (rospy.Time.now() - self._command_init_time).to_sec()  
					if t_diff > self._command_timeout:
						rospy.loginfo('%s::readyState: Timeout (%d secs) in command %s'%(self.node_name, t_diff, self._platform_current_command.command))
						self._crane_move_client.cancel()
						self.command_state = COMMAND_STATE_ENDED
						self._platform_current_command.command = FAIL_MOVE_CRANE
						
			# FOLD CRANE
			elif self._platform_current_command.command == COMMAND_FOLD_CRANE:
				traj_planner_state = self._crane_move_client.getState()
				
				if traj_planner_state.state.state == State.STANDBY_STATE and traj_planner_state.goal_state == 'IDLE':
					self.command_state = COMMAND_STATE_ENDED
					rospy.loginfo('%s::readyState: command %s finished'%(self.node_name,self._platform_current_command.command))
					self._platform_current_command.command = DONE_FOLD_CRANE
				else:
					t_diff = (rospy.Time.now() - self._command_init_time).to_sec()  
					if t_diff > self._command_timeout:
						rospy.loginfo('%s::readyState: Timeout (%d secs) in command %s'%(self.node_name, t_diff, self._platform_current_command.command))
						self._crane_move_client.cancel()
						self.command_state = COMMAND_STATE_ENDED
						self._platform_current_command.command = FAIL_FOLD_CRANE
			else:
				rospy.loginfo('%s::readyState: command %s disabled'%(self.node_name,self._platform_current_command.command))
				self.command_state = COMMAND_STATE_ENDED
				self._platform_current_command.command = ''
				
		
		elif self.command_state == COMMAND_STATE_ENDED:
			self._platform_current_command.variables = []
			self.command_state = COMMAND_STATE_INIT
			self.switchToState(State.STANDBY_STATE)
		
		
		return
		
	
	def shutdownState(self):
		'''
			Actions performed in shutdown state 
		'''
		if self.shutdown() == 0:
			self.switchToState(State.INIT_STATE)
		
		return
	
	
	def emergencyState(self):
		'''
			Actions performed in emergency state
		'''
		
		return
	
	
	def failureState(self):
		'''
			Actions performed in failure state
		'''
		
			
		return
	
	
	def switchToState(self, new_state):
		'''
			Performs the change of state
		'''
		if self.state != new_state:
			self.previous_state = self.state
			self.state = new_state
			rospy.loginfo('%s::switchToState: %s'%(self.node_name, self.stateToString(self.state)))
		
		return
	
		
	def allState(self):
		'''
			Actions performed in all states
		'''
		# check the communication with all the components
		# platform_controller?
		# navigation_planner?
		# trajectory_planner?
		# localization node? map->odom transform?
		
		
		self._update_platform_state()
		
		self.rosPublish()
		
		return
	
	
	def stateToString(self, state):
		'''
			@param state: state to set
			@type state: State
			@returns the equivalent string of the state
		'''
		if state == State.INIT_STATE:
			return 'INIT_STATE'
				
		elif state == State.STANDBY_STATE:
			return 'STANDBY_STATE'
			
		elif state == State.READY_STATE:
			return 'READY_STATE'
			
		elif state == State.EMERGENCY_STATE:
			return 'EMERGENCY_STATE'
			
		elif state == State.FAILURE_STATE:
			return 'FAILURE_STATE'
			
		elif state == State.SHUTDOWN_STATE:
			return 'SHUTDOWN_STATE'
		else:
			return 'UNKNOWN_STATE'
	
		
	def publishROSstate(self):
		'''
			Publish the State of the component at the desired frequency
		'''
		self.msg_state.state = self.state
		self.msg_state.state_description = self.stateToString(self.state)
		self.msg_state.desired_freq = self.desired_freq
		self.msg_state.real_freq = self.real_freq
		self._state_publisher.publish(self.msg_state)
		
		self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
		self.t_publish_state.start()
	
	
	def _platform_command_cb(self, req):
		'''
			Callback for commands
			@param req: received request
			@type msg: robospect_msgs/PlatformCommandSrv
		'''
		
		if self.state == State.STANDBY_STATE  and len(self._platform_commands) == 0:
			# Checks if the command is correct
			if not self._check_command(req.command):
				return "ERROR_COMMAND"
			if req.command.command != COMMAND_CANCEL:
				print req.command
				# Adding new command
				self._platform_commands.append(req.command)
				self._platform_command_time = rospy.Time.now()
		
			return "OK"
		else:
			if req.command.command == COMMAND_CANCEL:
				self._cancel_command = True
				return "OK"
			else:	
				return "BUSY"
		
		# Confirms the command reception, sending it back
		# self._platform_response_pub.publish(msg)
	
	def _odom_cb(self, msg):
		'''
			Callback for Odometry msg
			@param msg: received message
			@type msg: nav_msgs/Odometry
		'''
		self._odometry_time = rospy.Time.now()
		self._odometry = msg
		
	
	def _joint_state_cb(self, msg):
		'''
			Callback for JointState msg
			@param msg: received message
			@type msg: sensor_msgs/JointState
		'''
		self._joint_state_time = rospy.Time.now()
		self._joint_state = msg
		# Updates joint states
		for joint in range(len(msg.name)):
			try:
				joint_name = msg.name[joint]
			
				if joint_name in self._joints_dict:
					self._joints_dict[joint_name] = msg.position[joint]
				elif joint_name == self._joint_linear_speed_name:
					self._linear_speed = msg.velocity[joint]
					
			except IndexError, e:
				rospy.logerr('%s::_joint_state_cb: index %d, %s'%(self.node_name, joint, e))
		
			

	def _robot_state_cb(self, msg):
		'''
			Callback for State msg
			@param msg: received message
			@type msg: robotnik_msgs/State
		'''
		self._robot_state_time = rospy.Time.now()
		self._robot_state = msg
		
	
	def _check_command(self, command):
		'''
			Checks that the command received is correct
			@param command: Command to proces
			@type command: robospect_msgs/PlatformCommand
			@return True if the command is correct, False otherwise
		'''
		if command.command == COMMAND_ADVANCE:
			if len(command.variables) < 1:
				rospy.logerr('%s::_check_command: the command %s needs 1 variable for the distance',self.node_name, command.command)
				return False
			return True
			
		elif command.command == COMMAND_MOVE_CRANE:
			if len(command.variables) < 3:
				rospy.logerr('%s::_check_command: the command %s needs 3 variables for the x,y,z',self.node_name, command.command)
				return False
			return True
			
		elif command.command == COMMAND_FOLD_CRANE:
			return True
		elif command.command == COMMAND_CANCEL:
			return True
		else:
			return False
		
		
	def _command_to_string(self, command):
		'''
			Converts the command in a formatted string [command + variables]
			@param command: Command to convert
			@type command: robospect_msgs/PlatformCommand
		'''
		
		if command.command == COMMAND_ADVANCE:
			return '%s [%.2f m]'%(command.command, command.variables[0])
			
		if command.command == DONE_ADVANCE or command.command == FAIL_ADVANCE:
			return '%s'%(command.command)
		
		elif command.command == COMMAND_MOVE_CRANE:
			return '%s [%.2f %.2f %2f]'%(command.command, command.variables[0], command.variables[1], command.variables[2])
			
		elif command.command == DONE_MOVE_CRANE or command.command == FAIL_MOVE_CRANE:
			return '%s'%(command.command)
			
		elif command.command == COMMAND_FOLD_CRANE or command.command == DONE_FOLD_CRANE or command.command == FAIL_FOLD_CRANE:
			return command.command
		
		return ""	
		
	"""
	def serviceCb(self, req):
		'''
			ROS service server
			@param req: Required action
			@type req: std_srv/Empty
		'''
		# DEMO
		rospy.loginfo('RobospectPlatformMissionManager:serviceCb')	
	"""	
		
def main():

	rospy.init_node("robospect_platform_mission_manager")
	
	
	_name = rospy.get_name().replace('/','')
	
	arg_defaults = {
	  'topic_state': 'state',
	  'desired_freq': DEFAULT_FREQ,
	  'odom_frame_id': '/odom',
	  'map_frame_id': '/map',
	  'base_frame_id': '/base_footprint',
	  'joint_states_topic': '/joint_states',
	  'odom_topic': '/odom',
	  'robot_state_topic': '/robospect_platform_controller/state',
	  'crane_joints': ['crane_first_joint','crane_second_joint'],
	  'crane_tip_frame_id': '/tip_link',
	  'arm_frame_id': '/arm_link',
	  'camera_frame_id': '/grasshopper3_left_camera_lens_link',
	  'joint_linear_speed': 'j9_velocity',
	  'publish_mission_state': False,
	  'base_planner_name': '/robospect_planner',
	  'advance_speed': COMMAND_ADVANCE_SPEED,
	  'advance_max_speed': COMMAND_ADVANCE_MAX_SPEED,
	  'relative_navigation': True,
	  'trajectory_planner_state_topic': '/rt_traj_planner/state',
	  'trajectory_planner_cartesian_command_topic': '/rt_traj_planner/commands/cartesian_euler',
	  'trajectory_planner_joint_command_topic': '/rt_traj_planner/commands/joint_by_joint',
	  'crack_approach_arm_frame_id': '/arm_link',
	  'crack_approach_crane_frame_id': '/tip_link',
	  'crack_frame_id': '/map',
	  'traj_exec_actions_name': '/rt_traj_exe/actions',
	  'avoid_crane_movement': False
	}
	
	args = {}
	
	for name in arg_defaults:
		try:
			if rospy.search_param(name): 
				args[name] = rospy.get_param('~%s'%(name)) # Adding the name of the node, because the para has the namespace of the node
			else:
				args[name] = arg_defaults[name]
			#print name
		except rospy.ROSException, e:
			rospy.logerr('%s: %s'%(e, _name))
			
	
	rc_node = RobospectPlatformMissionManager(args)
	
	rospy.loginfo('%s: starting'%(_name))

	rc_node.start()


if __name__ == "__main__":
	main()
