#!/usr/bin/env python2
import math
# ROS
import rospy
from roslib import message
# movement
from geometry_msgs.msg import Twist

class Direction(object):
	LEFT = 1,
	RIGHT = 2,
	FORWARD = 3,
	BACKWARD = 4

class Move_Manager(object):
	def __init__(self, sensor_manager, min_dist=50, x=1, z=.5, delay=.5, rate=10, hardcode_x = 12):
		self.sm = sensor_manager
		self.tw_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
		self.min_dist = min_dist
		self.z = z
		self.x = x
		self.hx = hardcode_x
		self.rate = rospy.Rate(rate)

	# look right, check distance to wall, return left
	def check(self, direction):
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
			self.send_twist(self.x, 0)
		elif direction == Direction.BACKWARD:
			self.send_twist(-self.x, 0)
		else:
			self.turn(direction, hardcode)

	def turn(self, direction, hardcode):
		if hardcode:
			val = self.hx if direction == Direction.RIGHT else -self.hx
			for i in range(self.hx):
				self.send_twist(0, val)
		else:
			## Odometry is not as good as I imagined
			# use odometry sensors
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
	
	def stop(self):
		twist = Twist()
		self.tw_pub.publish(twist)

	def send_twist(self, x, z):
		twist = Twist()
		twist.linear.x = x
		twist.angular.z = z
		self.tw_pub.publish(twist)
		# TODO helps maybe?
		self.rate.sleep()
		self.stop()