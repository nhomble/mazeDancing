#!/usr/bin/env python2
import math
import rospy
from roslib import message

from nav_msgs.msg import Odometry

from sensor_msgs.msg import PointCloud2, PointField

class Sensor_Manager(object):
	def __init__(self):
		# ODOM things
		self._odom_node = rospy.init_node('odometry', anonymous=True)
		self._odom_subscriber = rospy.Subscriber('odom', Odometry, self.odom_callback)
		self.last_odom = None
		# PCL things
		self._pcl_subscriber = rospy.Subscriber('camera/depth_registered/points', PointCloud2, self.pcl_callback)
		self.last_pcl = None

	# just save data in memory to call later
	def odom_callback(self, data):
		self.last_odom = data
	def pcl_callback(self, data):
		rospy.loginfo("pcl callback")
		self.last_pcl = data

	# on demand get the sensor readings
	def curr_angle(self):
		if self.last_odom is None:
			return None, None
		orientation = self.last_odom.pose.pose.orientation
		lar = 2 * math.acos(orientation.w)
		lad = lar * 180 / math.pi
		return lar, lad
	
	def read_depth(self):
		if self.last_pcl is None:
			return None
		width = self.last_pcl.width/2
		height = self.last_pcl.height/2
		data_out = pc2.read_points(self.last_pcl, field_names=None, skip_nans=False, uvs=[width, height])
		int_data = next(data_out)
		return int_data
