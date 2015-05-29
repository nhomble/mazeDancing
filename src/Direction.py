#!/usr/bin/env python2

class Direction(object):
	LEFT = 0,
	RIGHT = 1,
	FORWARD = 2,
	BACKWARD = 3,
	TURN = {
		(0,):{
			(0,): (3,),
			(1,): (2,)
		},
		(1,):{
			(0,):(2,),
			(1,):(3,)
		},
		(2,):{
			(0,):(0,),
			(1,):(1,)
		},
		(3,):{
			(0,):(1,),
			(1,):(0,)
		}
	}
