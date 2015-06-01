#!/usr/bin/env python2

MIN_FORWARD_DIST = .75
MIN_TURN_DIST = .6
TWIST_X = 1
TWIST_Z = 2
RATE = 10
DELAY = 1
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
