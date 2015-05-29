#!/usr/bin/env python2
import time
import math
# ROS
import rospy
from roslib import message
# movement
from geometry_msgs.msg import Twist
from Direction import *
class Move_Manager(object):
	def __init__(self, sensor_manager, min_dist=.75, x=1, z=1, delay=1, rate=10, hardcode_x=12):
		# TODO do I need to create another ros node here?
		self.sm = sensor_manager

		self.tw_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
		self.min_dist = min_dist
		self.z = z
		self.x = x
		self.hx = hardcode_x
		self.rate = rospy.Rate(rate)
		self.delay = delay

	# look right, check distance to wall, return left
	def _check(self, direction):
		if direction == Direction.RIGHT:
			self.right()
			result = self.min_dist < self.sm.read_depth()
			self.left()
		elif direction == Direction.LEFT:
			self.left()
			result = self.min_dist < sm.read_depth()
			self.left()
		elif direction == Direction.FORWARD:
			result = self.min_dist < self.sm.read_depth()
		else:
			rospy.log("you asked to check backwards?")
		return result
	
	# we always turn orthogonally so we won't ask for z input
	def move(self, direction, hardcode=True):
		if direction == Direction.FORWARD:
			self._send_twist(self.x, 0)
		elif direction == Direction.BACKWARD:
			self._send_twist(-self.x, 0)
		elif direction == Direction.RIGHT or direction == Direction.LEFT:
			self._turn(direction, hardcode)
		else:
			rospy.loginfo("invalid direction to move()")
		# NOTE this should be the only place where we delay after movement
		time.sleep(self.delay)

	def _turn(self, direction, hardcode):
		# HACK we just hardcode a fixed number of identical twist messages to do
		# an orthogonal turn on a flat surface
		if hardcode:
			val = -self.hx if direction == Direction.RIGHT else self.hx
			for i in range(self.hx):
				self._send_twist(0, val)
		else:
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
	
	# halt movement of the turtlebot immediately
	def stop(self):
		twist = Twist()
		self.tw_pub.publish(twist)

	# actually send message
	def _send_twist(self, x, z):
		twist = Twist()
		twist.linear.x = x
		twist.angular.z = z
		self.tw_pub.publish(twist)
