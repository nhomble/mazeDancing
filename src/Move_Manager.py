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

# NOTE MUST BE imported by a ROS NODE
class Move_Manager(object):
	def __init__(self):
		self._tw_pub = rospy.Publisher(TWIST_PUB, Twist)
		self._rate = rospy.Rate(RATE)
		# FULL is full average
		# _AVG is just average of some partitions
		# Direction.* is the minimum distance in select partitions
		# look at _scan()
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
	
	def debug(self, a=1):
		print(self._checks)
	
	# 1: right
	# 2: left
	# 3: both
	# just record that we hit something
	def _collision(self, data):
		collisions = data.bumps_wheeldrops
		if collisions == 0 or self._last_collision is not None:
			return
		self.stop()
		self._send_twist(-COLLISION_X, 0)
		self._last_collision = collisions
	
	# executed when we have the chance
	def _handle_collision(self):
		self.maze.collision()
		self._last_collision = None

	# LaserScan callback
	def _scan(self, data):
		arr = data.data
		self._checks["FULL"] = arr[-1]
		length = len(arr) - 1
		part = length // 3
		self._checks[Direction.RIGHT] = min(arr[0:part])
		self._checks[Direction.FORWARD] = min(arr[part:2*part])
		self._checks[Direction.LEFT] = min(arr[2*part:3*part-1])
		self._checks["R_AVG"] = sum(arr[0:part])/part
		self._checks["L_AVG"] = sum(arr[2*part:3*part])/part
		self._checks["M_AVG"] = sum(arr[part:2*part])/part
		self._checks["ARRAY"] = arr
	
	# try to adjust the left and right side to be equidistant from an imaginary wall
	def center(self, count=1):
		if  self._checks["ARRAY"] is None or \
			self._checks["M_AVG"] > MIN_DIST or \
			count > CENTER_MAX_COUNT:
			return

		num = count + 1
		diff = self._checks["L_AVG"] - self._checks["R_AVG"]

		# avoid adjustements that are too big (possibly just bad detection)
		# avoid minute adjustements
		if  abs(diff) > CENTER_MAX_COUNT * CENTER_INC or \
			abs(diff) < CENTER_TURN_DIST:
			return

		# HACK
		# guessing here, but we need to make a better decision here
		# should the robot turn towards or away from a wall?
		coeff = 1
		if abs(diff) > CENTER_MAX_COUNT * CENTER_INC / 2:
			coeff = -1

		self._send_twist(CENTER_FORWARD, 0)
		self._send_twist(0, coeff * CENTER_INC)
		self.center(count=num)
	
	# we don't want to be too close to a wall, we want to be in the center of a "cell"
	def not_too_close(self, count=1):
		if self._checks["M_AVG"] < TOO_CLOSE and count < CLOSE_MAX_COUNT:
			self._send_twist(-BACKWARDS_X(CLOSE_INC), 0)
			c = count + 1
			self.not_too_close(count=c)

	# turning is left to the callee!
	# if True, then the robot has enough room to perform the directional movement
	def check(self, direction, num=1):
		rospy.loginfo("M: {} L: {} R: {}".format(self._checks["M_AVG"], self._checks["L_AVG"], self._checks["R_AVG"]))
		if direction == Direction.FORWARD:
			self.center()
		# check that we have room
		if self._checks[Direction.FORWARD] > MIN_DIST or self._checks["M_AVG"] > MIN_DIST:
			rospy.loginfo("middle is good")
			# ok but are the sides confident?
			key = "L_AVG" if direction == Direction.LEFT else "R_AVG"
			if self._checks[key] > MIN_DIST:
				rospy.loginfo("side is confidence")
				# good to go
				return True
			else:
				## need to correct, 
				## lets nudge forward since we might be looking at a corner
				# if we are trying to go forward then this check is irrelevant 
				if direction == Direction.FORWARD:
					return False
				# reset forward, move up a little, look left again
				# recursively check again
				# TODO depends which side sensor failed check!!!
				elif direction == Direction.LEFT:
					self.move(Direction.RIGHT)
					self.move(Direction.FORWARD, scale=CHECK_SCALE)
					self.move(Direction.LEFT)
					return self.check(direction)
				# ^^
				elif direction == Direction.RIGHT:
					self.move(Direction.LEFT)
					self.move(Direction.FORWARD, scale=CHECK_SCALE)
					self.move(Direction.RIGHT)
					return self.check(direction)
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
			self._send_twist(TWIST_X/scale, 0)
			self.maze.step()
			self.center()
			self.not_too_close()
		elif direction == Direction.BACKWARD:
			# NOTE: backward is not just -TWIST_X
			self._send_twist(-BACKWARD(MAX_DIST)/scale, 0)
		elif direction == Direction.RIGHT or direction == Direction.LEFT:
			self._turn(direction, hardcode)
			self.maze.turn(direction)
			self.center()
		else:
			rospy.loginfo("invalid direction to move() " + str(direction))

	# halt movement of the turtlebot immediately
	def stop(self):
		twist = Twist()
		self._tw_pub.publish(twist)

	def _turn(self, direction, hardcode):
		# an orthogonal turn on a flat surface
		# HACK right turns suck
		val = -(TWIST_Z*RIGHT_SCALE) if direction == Direction.RIGHT else TWIST_Z
		self._send_twist(0, val)

	# actually send message
	# similar to what is found in turtlebot min_max tutorial
	def _send_twist(self, x, z):
		for _ in range(TWIST_NUM):
			twist = Twist()
			twist.linear.x = x
			twist.angular.z = z
			self._tw_pub.publish(twist)
			self._rate.sleep()
		self.stop()
		rospy.sleep(DELAY)
