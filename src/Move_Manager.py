#!/usr/bin/env python2

import time
import math

# ROS
import rospy
from roslib import message

# movement
from geometry_msgs.msg import Twist
from consts import *

# NOTE MUST BE import by a ROS NODE
class Move_Manager(object):
	def __init__(self):
		self.tw_pub = rospy.Publisher(TWIST_PUB, Twist)

		# delays
		self.rate = rospy.Rate(RATE)

	'''
	# look right, check distance to wall, return left
	# return True if should/can move there
	# this is not in Sensor since we have to move the bot to do the checks
	def check(self, direction):
		result = None
		dist = None
		if direction == Direction.RIGHT:
			self.move(Direction.RIGHT)
			while dist is None:
				dist = self.sm.bias_pcl(direction)
			result = self.min_turn_dist < dist
			self.move(Direction.LEFT)
		elif direction == Direction.LEFT:
			self.move(Direction.LEFT)
			while dist is None:
				dist = self.sm.bias_pcl(direction)
			result = self.min_turn_dist < dist
			self.move(Direction.RIGHT)
		elif direction == Direction.FORWARD:
			while dist is None:
				dist = self.sm.bias_pcl(direction)
			result = self.min_forward_dist < dist
		else:
			rospy.log("you asked to check backwards?")
		return result
	'''
	
	# we always turn orthogonally so we won't ask for z input
	def move(self, direction, hardcode=True):
		# NOTE this should be the only place where we delay after movement
		if direction == Direction.FORWARD:
			self._send_twist(-TWIST_X, 0)
		elif direction == Direction.BACKWARD:
			self._send_twist(TWIST_X, 0)
		elif direction == Direction.RIGHT or direction == Direction.LEFT:
			self._turn(direction, hardcode)
		else:
			rospy.loginfo("invalid direction to move() " + str(direction))
		time.sleep(2)

	# halt movement of the turtlebot immediately
	def stop(self):
		twist = Twist()
		self.tw_pub.publish(twist)

	def _turn(self, direction, hardcode):
		# HACK we just hardcode a fixed number of identical twist messages to do
		# an orthogonal turn on a flat surface
		val = TWIST_Z if direction == Direction.RIGHT else -TWIST_Z
		if hardcode:
			for i in range(TWIST_TURN_NUM):
				self._send_twist(0, val)
		else:
			return # code is broken below
			# NOTE Odometry is not as good as I imagined
			_, curr_angle = self.sm.curr_angle()
			if curr_angle is None:
				return
			curr_angle = round(curr_angle)
			goal_angle = (curr_angle + 90) % 360
			# if we move slow enough this could work..
			# but this is dangerous
			initial_sign = (goal_angle - curr_angle) > 0
			val = self.z if direction == Direction.RIGHT else -self.y
			# look for sign change
			while initial_sign == (goal_angle - curr_angle > 0):
				self.move(0, val)
				_, curr_angle = self.sm.curr_angle()
				curr_angle = round(curr_angle)

	# actually send message
	def _send_twist(self, x, z):
		twist = Twist()
		twist.linear.x = x
		twist.angular.z = z
		self.tw_pub.publish(twist)
