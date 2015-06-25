#!/usr/bin/env python2

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
			Direction.FORWARD: Direction.FORWARD,
			Direction.BACKWARD: Direction.BACKWARD
		},
		Direction.BACKWARD: {
			Direction.LEFT: Direction.LEFT,
			Direction.RIGHT: Direction.RIGHT,
			Direction.FORWARD: Direction.FORWARD,
			Direction.BACKWARD: Direction.BACKWARD
		}
	}
	POS_FROM_OD = {
		Direction.LEFT: {
			Direction.LEFT: (1, 0),
			Direction.RIGHT: (-1, 0),
			Direction.FORWARD: (0, -1),
			Direction.BACKWARD: (0, 1)
		},
		Direction.RIGHT: {
			Direction.LEFT: (-1, 0),
			Direction.RIGHT: (1, 0),
			Direction.FORWARD: (0, 1),
			Direction.BACKWARD: (0, -1)
		},
		Direction.FORWARD:{
			Direction.LEFT: (-1, 0),
			Direction.RIGHT: (1, 0),
			Direction.FORWARD: (-1, 0),
			Direction.BACKWARD: (1, 0)
		},
		Direction.BACKWARD:{
			Direction.LEFT: (1, 0),
			Direction.RIGHT: (-1, 0),
			Direction.FORWARD: (1, 0),
			Direction.BACKWARD: (-1, 0)
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
