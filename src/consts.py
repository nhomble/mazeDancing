#!/usr/bin/env python2
import math

# =================== user define ==========================

# LaserScan detection
NUM_LASER_PARTITIONS = 8
DEG_ANGLE = 10

# movement primitive
TWIST_X = .20

# ROS topics
BUMPER_IO = '/mobile_base/sensors/core'
SPEECH_NODE = 'espeak_node'
SPEECH_IO = 'espeak_sub'
SCAN_IO = 'scan_dat'
SENSOR_NODE = 'sensor_manager'
TWIST_PUB = 'cmd_vel_mux/input/teleop'

# timing
DELAY = 1				# hard 1 second delay
DANCE_DELAY = 5

# feedback systems
CHECK_SCALE = 10
CENTER_MAX_COUNT = 3

# ===================== fixed  ================================

def x_to_d(x):
	return (3.535 * x) - .37

def BACKWARDS_X(dist):
	return (-abs(dist) + .019) / -2.972

RAD_ANGLE = DEG_ANGLE * math.pi / 180

# taken from ROS mini_max tutorial
TWIST_NUM = 20
RATE = 5.0
TIME = TWIST_NUM / RATE
TWIST_Z = 1.8/TIME		

TWIST_X = max(min(TWIST_X, .3), .1)			# ensure bounds
MIN_DIST = x_to_d(TWIST_X) * 1.1			# data collected by hand and fitted
											# give a 10 percent error buffer

# we don't to correct more than a quarter of the given travel distance
CHECK_MAX_COUNT = min(1, round((MIN_DIST / 4) / x_to_d(TWIST_X / CHECK_SCALE)))

CLOSE_INC = x_to_d(TWIST_X) * .05			# backwards distance we increment when too close
TOO_CLOSE = x_to_d(TWIST_X) * .1			# want to maintain a ten percent buffer
CLOSE_MAX_COUNT = TOO_CLOSE / CLOSE_INC		# max number of backwards we should do


CENTER_INC = .1								# small value that is consistent
CENTER_TURN_DIST = .08						#
CENTER_FORWARD = (.018 + .37) / 3.535		# when we turn, we do nudge backwards a little bit
# HACK
CENTER_FORWARD = 0

COLLISION_X = BACKWARDS_X(MIN_DIST)			# just reset movement, but not all the way

RIGHT_SCALE = .94							# for some reason right turn > left turn

# ===================== enums ===================================
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
	ERROR = 4
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
	to_string = {
		(0,): "LEFT",
		(1,): "RIGHT",
		(2,): "FORWARD",
		(3,): "BACKWARD",
		(4,): "ERROR",
		None: "NONE"
	}

# internal maze representation
class Maze_Cell(object):
	UNKNOWN = 1,
	OPEN = 2,
	WALL = 3

	CELL_TO_DIR = [
		[
			Direction.ERROR, 	# 0, 0
			Direction.FORWARD,	# 0, 1
			Direction.BACKWARD	# 0, -1
		],
		[
			Direction.RIGHT,	# 1, 0
			Direction.ERROR,	# 1, 1
			Direction.ERROR		# 1, -1
		],
		[
			Direction.LEFT,		# -1, 0
			Direction.ERROR,	# -1, 1
			Direction.ERROR		# -1, -1
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
	CLOCKWISE = -1,
	COUNTER = -2,
	to_string = {
		(-1,): "clockwise",
		(-2,): "counter clockwise"
	}

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
