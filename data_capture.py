#!/usr/bin/env python2

import math
import time
import sys

from consts import *

import rospy
from roslib import message

from nav_msgs.msg import Odometry

from sensor_msgs.msg import PointCloud2, PointField, LaserScan
import point_cloud2 as pc2
from std_msgs.msg import Float64, Int64, String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import SimpleCV
import numpy as np

rospy.init_node('data_capture')

val = 0

def ls_cb(data):
	s = 0
	for d in data.ranges:
		if not math.isnan(s):
			s += d
	s /= 640
	global val
	val = s
	print(val)


rospy.Subscriber('scan', LaserScan, ls_cb)
tw_pub = rospy.Publisher(TWIST_PUB, Twist)
def _send_twist(x, z):
	for _ in range(TWIST_NUM):
		twist = Twist()
		twist.linear.x = x
		twist.angular.z = z
		tw_pub.publish(twist)
		time.sleep(.1)

f = open('data.txt', 'a')
for i in range(180):
	_send_twist(0, .1)
	string = "{}\n".format(val)
	f.write(string)
	print(string)
f.close()
