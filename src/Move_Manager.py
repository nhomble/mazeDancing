#!/usr/bin/env python2
import math
# ROS
import rospy
from roslib import message
# movement
from geometry_msgs.msg import Twist

class Move_Manager(object):
	def __init__(self, sensor_manager, min_dist=50, x=1, z=.5, delay=.5, rate=10):
		self.sm = sensor_manager
		self.tw_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
		self.min_dist = min_dist
		self.z = z
		self.x = x
		self.rate = rospy.Rate(rate)

	def check_right(self):
		self.right()
		result = self.min_dist < self.sm.read_depth()
		self.left()
		return result

	def check_left(self):
		self.left()
		result = self.min_dist < sm.read_depth()
		self.left()
		return result

	def check_forward(self):
		return self.min_dist < self.sm.read_depth()
	
	def forward(self):
		self.move(self.x, 0)

	def right(self):
		_, curr_angle = self.sm.curr_angle()
		if curr_angle is None:
			return
		curr_angle = round(curr_angle)
		goal_angle = (curr_angle + 90) % 360
		# if we move slow enough this could work..
		# but this is dangerous
		initial_sign = (goal_angle - curr_angle) > 0
		while initial_sign == (goal_angle - curr_angle > 0):
			print("{} {}".format(curr_angle, goal_angle))
			self.move(0, -self.z)
			_, curr_angle = self.sm.curr_angle()
			curr_angle = round(curr_angle)

	def left(self):
		_, curr_angle = self.sm.curr_angle()
		if curr_angle is None:
			return
		curr_angle = round(curr_angle)
		goal_angle = (curr_angle + 90) % 360
		# if we move slow enough this could work..
		# but this is dangerous
		while curr_angle != goal_angle:
			self.move(0, self.z)
			_, curr_angle = self.sm.curr_angle()
			curr_angle = round(curr_angle)

	def backward(self):
		self.move(self.x, 0)

	def move(self, x, z):
		twist = Twist()
		twist.linear.x = x
		twist.angular.z = z
		self.tw_pub.publish(twist)
		self.rate.sleep()
