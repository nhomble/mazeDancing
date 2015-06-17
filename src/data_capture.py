#!/usr/bin/env python2

import math
import time
import sys

from consts import *

import rospy
from roslib import message

from nav_msgs.msg import Odometry

from sensor_msgs.msg import PointCloud2, PointField
import point_cloud2 as pc2
from std_msgs.msg import Float64, Int64, String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import SimpleCV
import numpy as np

rospy.init_node('data_capture')

m_arr = [-1, -1, -1, -1]
p_arr = [-1, -1, -1, -1]

def p_m(data):
	global m_arr
	m_arr = data.data

def p_f(data):
	global p_arr
	p_arr = data.data

rospy.Subscriber(PCL_FULL_IO, Float64MultiArray, p_m)
rospy.Subscriber(PCL_MIDDLE_IO, Float64MultiArray, p_f)
time.sleep(5)
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
	_send_twist(0, .05)
	string = "{} {} {} {} {} {} {} {} {}".format(m_arr[0], m_arr[1], m_arr[2], m_arr[3], p_arr[0], p_arr[1], p_arr[2], p_arr[3], i)
	f.write(string)
	print(string)
