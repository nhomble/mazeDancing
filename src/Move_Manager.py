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
		self.rate = rospy.Rate(RATE)

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
	
	# we always turn orthogonally so we won't ask for z input
	def move(self, direction, hardcode=True):
		# NOTE this should be the only place where we delay after movement
		if direction == Direction.FORWARD:
			self._send_twist(TWIST_X, 0)
		elif direction == Direction.BACKWARD:
			self._send_twist(-TWIST_X, 0)
		elif direction == Direction.RIGHT or direction == Direction.LEFT:
			self._turn(direction, hardcode)
		else:
			rospy.loginfo("invalid direction to move() " + str(direction))

	# halt movement of the turtlebot immediately
	def stop(self):
		twist = Twist()
		self.tw_pub.publish(twist)

	def _turn(self, direction, hardcode):
		# HACK we just hardcode a fixed number of identical twist messages to do
		# an orthogonal turn on a flat surface
		val = TWIST_Z if direction == Direction.RIGHT else -TWIST_Z
		self._send_twist(0, val)

	# actually send message
	def _send_twist(self, x, z):
		for _ in range(TWIST_NUM):
			twist = Twist()
			twist.linear.x = x
			twist.angular.z = z
			rospy.loginfo("new twist message: " + str(twist))
			self.tw_pub.publish(twist)
			self.rate.sleep()
		self.stop()
		rospy.sleep(DELAY)
