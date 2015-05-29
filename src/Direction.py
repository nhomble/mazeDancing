#!/usr/bin/env python2

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
