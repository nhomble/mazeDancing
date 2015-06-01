#!/usr/bin/env python2

MIN_FORWARD_DIST = .75
MIN_TURN_DIST = .6
# taken from ROS mini_max tutorial
TWIST_X = .15		# .15 m/s
TWIST_Z = 1.57/2	# 45 deg/s * 2 sec = 90 degress
TWIST_NUM = 10		
RATE = 5.0			# 10 * 5 Hz = 2 sec
DELAY = 1			# hard 1 second delay

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

'''
	FORWARD
LEFT		RIGHT
	BACKWARD
'''

class Direction(object):
	LEFT = 0,
	RIGHT = 1,
	FORWARD = 2,
	BACKWARD = 3,
	TURN = {
		# LEFT
		(0,):{ 
			# LEFT turn --> BACKWARD
			(0,): (3,),
			# RIGHT turn --> FORWARD
			(1,): (2,)
		},
		# RIGHT
		(1,):{
			# LEFT turn --> FORWARD
			(0,):(2,),
			# RIGHT turn --> BACKWARD
			(1,):(3,)
		},
		# FORWARD
		(2,):{
			# LEFT turn --> LEFT
			(0,):(0,),
			# RIGHT turn --> RIGHT
			(1,):(1,)
		},
		# BACKWARD
		(3,):{
			# LEFT turn --> RIGHT
			(0,):(1,),
			# RIGHT turn --> LEFT
			(1,):(0,)
		}
	}
