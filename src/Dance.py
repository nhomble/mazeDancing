#!/usr/bin/env python2

from consts import *
import time

# execute movements
def do_circle(d, move):
	for _ in range(4):
		move.move(d)

# for each movement:
#	move in that direction for a whole circle
#	then pause  to delimit the directions
def do_dance(directions, move, follower=Follower.RIGHT):
	for d in directions:
		if d == Direction.LEFT:
			if follower is not Follower.LEFT:
				raise Exception("I cannot move left with right follower")
		elif d == Direction.RIGHT:
			if follower is not Follower.RIGHT:
				raise Exception("I cannot move right with left follower")
		elif d == Direction.FORWARD:
			d = TRANSLATE_FORWARD[follower]
		_do_circle(d, move)
		move.stop()
		time.sleep(1)

# TODO
# need to do image processing to detect movements
def interpret_dance():
	directions = []
	return directions
