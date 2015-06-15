#!/usr/bin/env python2

MAX_DIST = .90		# no further away to the wall than this empirically chosen value
CHECK_OPEN = 2.0
MAX_STD_DEV = 2.0
MAX_WITHIN_PERC = .95
MIN_POINTS = 50

# taken from ROS mini_max tutorial
TWIST_NUM = 10		
RATE = 5.0
TIME = TWIST_NUM / RATE
TWIST_X = .375			# .15 m/s
TWIST_Z = 1.70/TIME		# 45 (+ eps) deg/s * 2 sec = 90 degress
DELAY = 1				# hard 1 second delay

DANCE_DELAY = 3

MAX_NUDGE = .75

SPEECH_NODE = 'espeak_node'
SPEECH_IO = 'espeak_sub'

SENSOR_NODE = 'sensor_manager'
PCL_FULL_IO = 'pcl/full'
PCL_RIGHT_IO = 'pcl/right'
PCL_LEFT_IO = 'pcl/left'
PCL_MIDDLE_IO = 'pcl/middle'
PCL_VARIANCE = 'pcl/variance'

PCL_X_MIN = -1.0
PCL_Y_MIN = 0.3
PCL_X_MAX = 1.0
PCL_Y_MAX = 0.5
PCL_INTERVAL = 50

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
	DIR_TO_TURN = {
		Direction.LEFT: {
			Direction.LEFT: Direction.BACKWARD,
			Direction.RIGHT: Direction.FORWARD,
			Direction.FORWARD: Direction.LEFT,
			Direction.BACKWARD: Direction.RIGHT
		},
		Direction.RIGHT: {
			Direction.LEFT: Direction.FORWARD,
			Direction.RIGHT: Direction.BACKWARD,
			Direction.FORWARD: Direction.RIGHT,
			Direction.BACKWARD: Direction.LEFT
		},
		Direction.FORWARD: {
			Direction.LEFT: Direction.RIGHT,
			Direction.RIGHT: Direction.LEFT,
			Direction.FORWARD: Direction.BACKWARD,
			Direction.BACKWARD: Direction.FORWARD
		},
		Direction.BACKWARD: {
			Direction.LEFT: Direction.LEFT,
			Direction.RIGHT: Direction.RIGHT,
			Direction.FORWARD: Direction.FORWARD,
			Direction.BACKWARD: Direction.BACKWARD
		}
	}
	
# we can be a left or right wall follower
class Follower(object):
	LEFT = 1,
	RIGHT = 2,
	BOTH = 3

class Turn(object):
	CLOCKWISE = 1,
	COUNTER = 2

class Tag(object):
	START = 1,
	GOAL = 2,
	FRONT = 3,
	RIGHT = 4,
	LEFT = 5,
	BACK = 6,
	WORKER = 7				# allow the scout robot to detect worker robot
	@staticmethod
	def translate_id(num):
		return (num,)
	
# translate tag_id into orientations
# translate orientations into directions
#			back
#		    KINECT 
#
#   right			left 
#
#		FRONT OF BOT
#			front
class Language(object):
	# worker view
	# poses come in two positions
	# detect the rotation done
	ID_TO_CLOCK = {
		Tag.FRONT: {
			Tag.LEFT: Turn.CLOCKWISE,
			Tag.RIGHT: Turn.COUNTER
		},
		Tag.RIGHT: {
			Tag.FRONT: Turn.CLOCKWISE,
			Tag.BACK: Turn.COUNTER
		},
		Tag.LEFT: {
			Tag.BACK: Turn.CLOCKWISE,
			Tag.FRONT: Turn.COUNTER
		},
		Tag.BACK: {
			Tag.RIGHT: Turn.CLOCKWISE,
			Tag.LEFT: Turn.COUNTER
		}
	}

	# worker view
	# translate turning to actual movement
	CLOCK_TO_DIR = {
		Turn.CLOCKWISE: {
			Turn.CLOCKWISE: Direction.LEFT,
			Turn.COUNTER: Direction.RIGHT
		},
		Turn.COUNTER: {
			Turn.CLOCKWISE: Direction.FORWARD,
			Turn.COUNTER: None					# means we are done communication
		}
	}

	# scout view
	# translate movement to clock
	DIR_TO_CLOCK = {
		Direction.LEFT: (Turn.CLOCKWISE, Turn.CLOCKWISE),
		Direction.RIGHT: (Turn.CLOCKWISE, Turn.COUNTER),
		Direction.FORWARD: (Turn.COUNTER, Turn.CLOCKWISE),
		None: (Turn.COUNTER, Turn.COUNTER)
	}
