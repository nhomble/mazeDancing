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
		self._checks = {
			"FULL": None,\
			"L_AVG": None,\
			"R_AVG": None,\
			"M_AVG": None,\
			Direction.FORWARD: None,\
			Direction.LEFT: None,\
			Direction.RIGHT: None,\
			"ARRAY": None\
		}
		self._sense_subs = [\
			rospy.Subscriber(BUMPER_IO, TurtlebotSensorState, self._collision),\
			rospy.Subscriber(SCAN_IO, Float64MultiArray, self._scan)\
		]
		self.maze = Maze()
		self._last_collision = None
	
	# 1: right
	# 2: left
	# 3: both
	# just record that we hit something
	def _collision(self, data):
		collisions = data.bumps_wheeldrops
		if collisions == 0 or self._last_collision is not None:
			return
		self.stop()
		self._send_twist(-TWIST_X/7, 0)
		self._last_collision = collisions
	
	# executed when we have the chance
	def _handle_collision(self):
		self.maze.collision()
		#if self._last_collision == 1:
		#	self._send_twist(0, COLLISION_Z)
		#elif self._last_collision == 2:
		#	self._send_twist(0, -COLLISION_Z)
		self._last_collision = None

	def _scan(self, data):
		arr = data.data
		self._checks["FULL"] = arr[-1]
		length = len(arr) - 1
		part = length // 3
		self._checks[Direction.RIGHT] = min(arr[0:part])
		self._checks[Direction.FORWARD] = min(arr[part:2*part])
		self._checks[Direction.LEFT] = min(arr[2*part:3*part])
		self._checks["R_AVG"] = sum(arr[2*part:3*part])/part
		self._checks["L_AVG"] = sum(arr[0:part])/part
		self._checks["M_AVG"] = sum(arr[part:2*part])/part
		self._checks["ARRAY"] = arr
	
	# try to adjust the left and right side to be equidistant from an imaginary wall
	def center(self, count=1):
		if self._checks["ARRAY"] is None:
			return
		if self._checks["M_AVG"] > MIN_PART_DIST:
			return
		if count > CENTER_MAX_COUNT:
			return
		num = count + 1
		diff = self._checks["L_AVG"] - self._checks["R_AVG"]

		# avoid big adjustments
		if abs(diff) > CENTER_MAX_COUNT * CENTER_INC:
			return

		# NOTE
		# we aren't being as smart as we could be
		# we always try to move towards a wall..
		# but maybe we should turn the other way
		if diff < 0:
			self._send_twist(0, -CENTER_INC)
			self._send_twist(CENTER_INC, 0)
			self.center(count=num)
		elif diff > 0:
			self._send_twist(0, CENTER_INC)
			self._send_twist(CENTER_INC, 0)
			self.center(count=num)
	
	def not_too_close(self, count=1):
		if self._checks["FULL"] < MIN_FULL_DIST/8 and count < 2:
			self.move(Direction.BACKWARD, scale=10)
			c = count + 1
			self.not_too_close(count=c)
		

	# turning is left to the callee!
	# if True, then the robot has enough room to perform the directional movement
	def check(self, direction):
		if direction == Direction.FORWARD:
			self.center()
			self.not_too_close()
		if self._checks[Direction.FORWARD] > MIN_PART_DIST or self._checks["FULL"] > MIN_FULL_DIST:
			# ok but are the sides confident
			if self._checks[direction] > MIN_PART_DIST:
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
		if self._last_collision is not None:
			self._handle_collision()

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
			self.center()
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
		scale = 1
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
