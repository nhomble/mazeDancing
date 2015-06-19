#!/usr/bin/env python2

import time
import math

# ROS
import roslib; roslib.load_manifest('mazeDancing')
import rospy

# movement
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
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
			"L": None,\
			"R": None,\
			Direction.FORWARD: None,\
			Direction.LEFT: None,\
			Direction.RIGHT: None\
		}
		self._sense_subs = [\
			rospy.Subscriber('/mobile_base/sensors/core', TurtlebotSensorState, self._collision),\
			rospy.Subscriber(SCAN_IO, Float64MultiArray, self._scan)\
		]
		self.maze = Maze()
		self.fix_bumper = (False, False)
	
	# 1: right
	# 2: left
	# 3: both
	def _collision(self, data):
		collisions = data.bumps_wheeldrops
		if collisions == 0:
			return
		self.stop()
		self._sense_subs[0].unregister()
		self.maze.collision()
		self._send_twist(-TWIST_X/7, 0)
		if collisions == 1:
			self._send_twist(0, .1)
		elif collisions == 2:
			self._send_twist(0, -.1)
		rospy.loginfo("collision: " + str(collisions))
		self._sense_subs[0] = rospy.Subscriber('/mobile_base/sensors/core', TurtlebotSensorState, self._collision)
	
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

	def _pcl_full(self, data):
		if data is None:
			rospy.loginfo("data is none")
			return
		pcl = data.data

		self._checks["FULL"] = pcl
	
	def _scan(self, data):
		arr = data.data
		self._checks["FULL"] = arr[-1]
		length = len(arr) - 1
		part = length // 3
		self._checks[Direction.RIGHT] = min(arr[0:part])
		self._checks[Direction.FORWARD] = min(arr[part:2*part])
		self._checks[Direction.LEFT] = min(arr[2*part:3*part])
		self._checks["R"] = max(arr[2*part:3*part])
		self._checks["L"] = max(arr[0:part])

	# TODO
	# broken
	# adjust a little bit towards the goal by MIN/MAX_FORWARD_DIST
	def nudge(self):
		return
		# NOTE most reliable measurement at the moment
		pos = self._checks["FULL"]
		diff = (GOAL_DIST - pos) / TIME
		# _send_twist takes 1 second to perform the entire movement
		# so we should not have to scale diff at all

		# boundary conditions
		diff = min(MAX_NUDGE, diff) if diff > 0 else max(-MAX_NUDGE, diff)
		rospy.loginfo("nudge: " + str(diff))
		self._send_twist(diff, 0)
	
	def center(self, count=1):
		if count > 4:
			return
		if self._checks["FULL"] > .8:
			return 

		num = count + 1
		diff = self._checks["L"] - self._checks["R"]
		rospy.loginfo("{} - {}  = diff {}".format(self._checks["L"], self._checks["R"], diff))
		if abs(abs(self._checks["L"] - self._checks[Direction.FORWARD]) - abs(self._checks["R"] - self._checks[Direction.FORWARD])) > .2:
			return
		if abs(diff) > .4:
			return
		if abs(diff) < .10:
			if diff < 0:
				self._send_twist(0, -.05)
				self.center(count=num)
			elif diff > 0:
				self._send_twist(0, .05)
				self.center(count=num)
		elif abs(diff) >= .2:
			if diff < 0:
				self._send_twist(0, .05)
				self.center(count=num)
			elif diff > 0:
				self._send_twist(0, -.05)
				self.center(count=num)
		

	# turning is left to the callee!
	# if True, then the robot has enough room to perform the directional movement
	def check(self, direction):
		if direction == Direction.FORWARD:
			self.center()
		if self._checks[Direction.FORWARD] > .65 or self._checks["FULL"] > .6:
			# ok but are the sides confident
			if self._checks[direction] > .6:
				return True
			else:
				# need to correct
				if direction == Direction.FORWARD:
					return False
				elif direction == Direction.LEFT:
					self.move(Direction.RIGHT)
					self.move(Direction.FORWARD, scale=5)
					self.move(Direction.LEFT)
					return self.check(direction)
				elif direction == Direction.RIGHT:
					self.move(Direction.LEFT)
					self.move(Direction.FORWARD, scale=5)
					self.move(Direction.RIGHT)
					return self.check(direction)
			return True
		else:
			rospy.loginfo("{} {}".format(self._checks["FULL"], self._checks[Direction.FORWARD]))
			return False

		rospy.loginfo(measure)
		if _check_dir(measure):
			if direction == Direction.RIGHT or direction == Direction.LEFT:
				measure = self._checks[direction]
				rospy.loginfo(measure)
				return _check_dir(measure)
			else:
				return True
		else:
			return False
	
	# we always turn orthogonally so we won't ask for z input
	def move(self, direction, hardcode=True, scale=1):
		if direction == Turn.CLOCKWISE:
			direction = Direction.RIGHT
		if direction == Turn.COUNTER:
			direction = Direction.LEFT


		# NOTE this should be the only place where we delay after movement
		if direction == Direction.FORWARD:
			self.center()
			self._send_twist(TWIST_X/scale, 0)
			self.maze.step()
		elif direction == Direction.BACKWARD:
			self._send_twist(-TWIST_X/scale, 0)
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
		scale = .92
		val = -(TWIST_Z*scale) if direction == Direction.RIGHT else TWIST_Z
		rospy.loginfo(Direction.to_string[direction] + " {}".format(val))
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
	
def _check_dir(measure):
	rospy.loginfo("measure: " + str(measure))
	if measure is None:
		rospy.loginfo("none")
		return True
	if measure[0] > CHECK_OPEN and measure[0] < 10:
		rospy.loginfo("open")
		return True
	elif measure[1] > MAX_STD_DEV:
		rospy.loginfo("std dev")
		return False
	elif measure[2] < MIN_POINTS:
		rospy.loginfo("points")
		return False
	elif measure[3]/measure[2] >= MAX_WITHIN_PERC:
		rospy.loginfo("perc")
		return False
	else:
		rospy.loginfo("reg")
		return measure[0] > MAX_DIST
