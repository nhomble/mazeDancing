#!/usr/bin/env python2

import time
import math

# ROS
import roslib; roslib.load_manifest('mazeDancing')
import rospy

# movement
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from consts import *
from create_node.msg import TurtlebotSensorState

# maze solving
from Maze import Maze

# NOTE MUST BE import by a ROS NODE
class Move_Manager(object):
	def __init__(self):
		self._tw_pub = rospy.Publisher(TWIST_PUB, Twist)
		self._rate = rospy.Rate(RATE)
		self._checks = {\
			"FULL": None,\
			Direction.FORWARD: None,\
			Direction.LEFT: None,\
			Direction.RIGHT: None\
		}
		# HACK
		# multiarrays are trickier than I would like to deal with right now..
		self._last_variance = None
		self._sense_subs = [\
			rospy.Subscriber(PCL_FULL_IO, Float64, self._pcl_full),\
			rospy.Subscriber(PCL_LEFT_IO, Float64, self._pcl_left),\
			rospy.Subscriber(PCL_RIGHT_IO, Float64, self._pcl_right),\
			rospy.Subscriber(PCL_MIDDLE_IO, Float64, self._pcl_middle),\
			rospy.Subscriber(PCL_VARIANCE, Float64, self._pcl_variance),\
			rospy.Subscriber('/mobile_base/sensors/core', TurtlebotSensorState, self._collision)\
		]
		self.maze = Maze()
	
	# 1: right
	# 2: left
	# 3: both
	def _collision(self, data):
		collisions = data.bumps_wheeldrops
		if collisions == 0:
			return
		rospy.loginfo("collision: " + str(collisions))
	
	def _pcl_variance(self, data):
		if data is None:
			rospy.loginfo("data is none")
		self._last_variance = data.data
	def _pcl_left(self, data):
		if data is None:
			rospy.loginfo("data is none")
			return
		pcl = data.data
		self._checks[Direction.LEFT] = pcl

	def _pcl_right(self, data):
		if data is None:
			rospy.loginfo("data is none")
			return
		pcl = data.data
		self._checks[Direction.RIGHT] = pcl

	def _pcl_middle(self, data):
		if data is None:
			rospy.loginfo("data is none")
			return
		pcl = data.data
		self._checks[Direction.FORWARD] = pcl

	# NOTE
	# misleading since this is an array, not float64
	def _pcl_full(self, data):
		if data is None:
			rospy.loginfo("data is none")
			return
		pcl = data.data
		rospy.loginfo(pcl)

		self._checks["FULL"] = pcl
	
	# adjust a little bit towards the goal by MIN/MAX_FORWARD_DIST
	def nudge(self):
		# NOTE most reliable measurement at the moment
		pos = self._checks["FULL"]
		diff = (GOAL_DIST - pos) / TIME
		# _send_twist takes 1 second to perform the entire movement
		# so we should not have to scale diff at all

		# boundary conditions
		diff = min(MAX_NUDGE, diff) if diff > 0 else max(-MAX_NUDGE, diff)
		rospy.loginfo("nudge: " + str(diff))
		self._send_twist(diff, 0)

	# turning is left to the callee!
	# if True, then the robot has enough room to perform the directional movement
	def check(self, direction):
		# NOTE the most reliable measurement at the moment
		measure = self._checks["FULL"]
		direction = Direction.FORWARD
		rospy.loginfo("right {} variance {} :: {} > {}".format(self._checks["RIGHT"], self._last_variance, measure, MAX_FORWARD_DIST))
		if measure > CHECK_OPEN:
			return True
		if self._last_variance > MAX_VARIANCE:
			return False
		return measure > MAX_FORWARD_DIST if direction == Direction.FORWARD else measure > MAX_TURN_DIST
	
	# we always turn orthogonally so we won't ask for z input
	def move(self, direction, hardcode=True):
		if direction == Turn.CLOCKWISE:
			direction = Direction.RIGHT
		if direction == Turn.COUNTER:
			direction = Direction.LEFT


		# NOTE this should be the only place where we delay after movement
		if direction == Direction.FORWARD:
			self._send_twist(TWIST_X, 0)
			self.maze.step()
		elif direction == Direction.BACKWARD:
			self._send_twist(-TWIST_X, 0)
			self.maze.step()
		elif direction == Direction.RIGHT or direction == Direction.LEFT:
			self._turn(direction, hardcode)
			self.maze.turn(direction)
		else:
			rospy.loginfo("invalid direction to move() " + str(direction))

	# halt movement of the turtlebot immediately
	def stop(self):
		twist = Twist()
		self._tw_pub.publish(twist)

	def _turn(self, direction, hardcode):
		# HACK we just hardcode a fixed number of identical twist messages to do
		# an orthogonal turn on a flat surface
		val = -TWIST_Z if direction == Direction.RIGHT else TWIST_Z
		self._send_twist(0, val)

	# actually send message
	def _send_twist(self, x, z):
		for _ in range(TWIST_NUM):
			twist = Twist()
			twist.linear.x = x
			twist.angular.z = z
			self._tw_pub.publish(twist)
			self._rate.sleep()
		self.stop()
		rospy.sleep(DELAY)
