#!/usr/bin/env python2

MIN_FORWARD_DIST = .75
MIN_TURN_DIST = .6
TWIST_X = 1
TWIST_Z = 1
RATE = 10
DELAY = 11
TWIST_TURN_NUM = 12

SPEECH_NODE = 'espeak_node'
SPEECH_IO = 'espeak_sub'

SENSOR_NODE = 'sensor_manager'
PCL_FULL_IO = 'pcl/full'
PCL_RIGHT_IO = 'pcl/right'
PCL_LEFT_IO = 'pcl/left'
PCL_MIDDLE_IO = 'pcl/middle'

ODOM_SUB = 'odom'
PCL_SUB = '/camera/depth/points'
DEPTH_SUB = '/camera/depth/image'
COLOR_SUB = '/camera/rgb/image_color'

TWIST_PUB = 'cmd_vel_mux/input/teleop'
