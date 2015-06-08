#!/usr/bin/env python2

MIN_FORWARD_DIST = .75	# no closer to the wall than this empirically chosen value
MAX_FORWARD_DIST = 0.9	# no further away to the wall than this empirically chosen value
MIN_TURN_DIST = .6		# ^ ditto but for turning
MAX_TURN_DIST = .8

# taken from ROS mini_max tutorial
TWIST_NUM = 10		
RATE = 5.0
TIME = TWIST_NUM / RATE
TWIST_X = .15			# .15 m/s
TWIST_Z = 1.57/TIME		# 45 deg/s * 2 sec = 90 degress
DELAY = 1				# hard 1 second delay

SPEECH_NODE = 'espeak_node'
SPEECH_IO = 'espeak_sub'

SENSOR_NODE = 'sensor_manager'
PCL_FULL_IO = 'pcl/full'
PCL_RIGHT_IO = 'pcl/right'
PCL_LEFT_IO = 'pcl/left'
PCL_MIDDLE_IO = 'pcl/middle'

PCL_X_MIN = -1.0
PCL_Y_MIN = 0.35
PCL_X_MAX = 1.0
PCL_Y_MAX = 1.2

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

# python2 enums
class Direction(object):
	LEFT = 0,
	RIGHT = 1,
	FORWARD = 2,
	BACKWARD = 3,
	ERROR = 4,
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

# internal maze representation
class Maze_Cell(object):
	UNKNOWN = 1,
	OPEN = 2,
	WALL = 3

	CELL_TO_DIR = [
		[
			Direction.ERROR,
			Direction.RIGHT,
			Direction.LEFT
		],
		[
			Direction.BACKWARD,
			Direction.ERROR,
			Direction.ERROR
		],
		[
			Direction.FORWARD,
			Direction.ERROR,
			Direction.ERROR
		]
	]
	
# we can be a left or right wall follower
class Follower(object):
	LEFT = 1,
	RIGHT = 2

	# depending on the follower, I will choose the direction accordngly
	TRANSLATE_FORWARD = {
		(1,0):Direction.RIGHT,
		(2,0):Direction.LEFT
	}
