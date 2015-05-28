#!/usr/bin/env python2
import math
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
		self.node = rospy.init_node('sensor_manager')

		# ODOM things
		self._odom_subscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)
		self.last_odom = None
		# PCL things
		self._pcl_subscriber = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.pcl_callback)
		self.last_pcl = None

		# Images
		self.last_depth_image = None
		self.last_color_image = None
		self._depth_subscriber = rospy.Subscriber('/camera/depth/image', Image, self.depth_callback)
		# TODO color

	# just save data in memory to call later
	def odom_callback(self, data):
		self.last_odom = data
	def pcl_callback(self, data):
		self.last_pcl = data
	def depth_callback(self, data):
		self.last_depth_image = data

	# on demand get the sensor readings
	def curr_angle(self):
		if self.last_odom is None:
			return None, None
		orientation = self.last_odom.pose.pose.orientation
		lar = 2 * math.acos(orientation.w)
		lad = lar * 180 / math.pi
		return lar, lad
	
	def full_pcl(self):
		if self.last_pcl is None:
			return None
		width = self.last_pcl.width
		height = self.last_pcl.height
		data_out = pc2.read_points(self.last_pcl, skip_nans=True)
		return _process_pcl(data_out)

	def bias_pcl(self, direction):
		if self.last_pcl is None:
			return None
		width = self.last_pcl.width
		height = self.last_pcl.height
		data_out = pc2.read_points(self.last_pcl, skip_nans=True)
		return _process_pcl(data_out)

	def full_depth(self):
		if self.last_depth_image is None:
			return
		b = CvBridge()
		depth_image = b.imgmsg_to_cv(self.last_depth_image, '32FC1')
		depth_array = np.array(depth_image, dtype=np.float32)

		accum = 0
		num = 0
		for row in depth_array:
			for ele in row:
				try:
					float(ele)
					accum += ele
					num += 1
				except:
					pass
		return accum / num if num != 0 else None
	def bias_depth(self, direction):
		if self.last_depth_image is None:
			return
		b = CvBridge()
		depth_image = b.imgmsg_to_cv(self.last_depth_image, '16UC1')
		depth = SimpleCV.Image(depth_image, cv2image=True)
		cut_point = 0 if direction == Direction.LEFT else width/2
		depth_cropped = depth.crop(cut_point, 0, depth.width/2, depth.height)
		flat = depth_cropped.getNumpy().flatten()
		accum = 0
		for i in flat:
			accum += flat[i]
		return accum / flat.length if flat.length > 0 else None

def _process_pcl(data):
	total = 0
	num = 0
	for point in data:
		total += point[2]
		num += 1
	return total/num if num != 0 else None
