#!/usr/bin/env python2
import math
import time
import rospy
from roslib import message

from nav_msgs.msg import Odometry

from sensor_msgs.msg import PointCloud2, PointField
import point_cloud2 as pc2

from Direction import *

import cv2
import cv2.cv as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import SimpleCV
import numpy as np

class Sensor_Manager(object):
	def __init__(self):
		# need to init a node to subscribe 
		self._node = rospy.init_node('sensor_manager')
		# ODOM things
		self._odom_subscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)
		self.last_odom = None
		# PCL things
		self._pcl_subscriber = rospy.Subscriber('/camera/depth/points', PointCloud2, self.pcl_callback)
		self.last_pcl = None

		# Images
		self.last_depth_image = None
		self.last_color_image = None
		self._depth_subscriber = rospy.Subscriber('/camera/depth/image', Image, self.depth_callback)
		self._color_subscriber = rospy.Subscriber('camera/rgb/image_color', Image, self.color_callback)
	
	def not_init(self):
		return  self.last_depth_image is None or \
				self.last_pcl is None or \
				self.last_color_image is None

	# just save data in memory to call later on demand
	def odom_callback(self, data):
		self.last_odom = data
	def pcl_callback(self, data):
		self.last_pcl = data
	def depth_callback(self, data):
		self.last_depth_image = data
	def color_callback(self, data):
		self.last_color_image = data

	# NOTE attempted to use odometry data to determine angle
	# but the calculations are surprisingly poor
	def curr_angle(self):
		if self.last_odom is None:
			return None, None
		orientation = self.last_odom.pose.pose.orientation
		lar = 2 * math.acos(orientation.w)
		lad = lar * 180 / math.pi
		return lar, lad
	
	# average ALL points in cloud
	def full_pcl(self):
		if self.last_pcl is None:
			return None
		width = self.last_pcl.width
		height = self.last_pcl.height
		data_out = pc2.read_points(self.last_pcl, skip_nans=True)
		return _process_pcl(data_out)

	# only look at a portion of the pcl depending on where I want to look
	def bias_pcl(self, direction):
		if self.last_pcl is None:
			return None
		width = self.last_pcl.width
		height = self.last_pcl.height
		validator = None
		# we want pcl focused in the middle, ignore surroundings
		if direction == Direction.FORWARD:
			validator = lambda point: True if point[0] > width/3 and \
			point[0] < 2 * width/3 and point[1] > height/3 and point[2] < 2 * height/3 else False

		elif direction == Direction.LEFT:
			validator = lambda point: True if point[0] <= width/2 else False
		elif direction == Direction.RIGHT:
			validator = lambda point: True if point[0] >= width/2 else False
		else:
			rospy.loginfo("you wanted pcl from behind?")
			return

		data_out = pc2.read_points(self.last_pcl, skip_nans=True)
		return _process_pcl(data_out, validator)

	# depracated
	# pcl takes care of the need to use /camera/depth/image
	def full_depth(self):
		if self.last_depth_image is None:
			return
		b = CvBridge()
		depth_image = b.imgmsg_to_cv(self.last_depth_image, '32FC1')
		depth_array = np.array(depth_image, dtype=np.float32)
		return None 
	def bias_depth(self, direction):
		if self.last_depth_image is None:
			return
		b = CvBridge()
		depth_image = b.imgmsg_to_cv(self.last_depth_image, '16UC1')
		depth = SimpleCV.Image(depth_image, cv2image=True)
		return None

# get depth from pcl if validator say the point is good
def _process_pcl(data, validator=lambda points: True):
	total = 0
	num = 0
	for point in data:
		if validator(point):
			total += point[2]
			num += 1
	return total/num if num != 0 else None
