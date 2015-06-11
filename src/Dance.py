#!/usr/bin/env python2

from consts import *
import time

# for each movement:
#	move in that direction for a whole circle
#	then pause  to delimit the directions
def do_dance(directions, move):
	for d in directions:
		clock = Language.DIR_TO_CLOCK[d]
		move.move(clock[0])
		time.sleep(2)
		move.move(clock[1])

def get_directions(turns):
	i = 2
	directions = []
	while i <= len(turns):
		couple = turns[i-2:i]
		directions.append(Language.CLOCK_TO_DIR[couple[0]][couple[1]])
		i += 2
	return directions

# TODO
# need to do image processing to detect movements
def interpret_dance():
	directions = []
	return directions

