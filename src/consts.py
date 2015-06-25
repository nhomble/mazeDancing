#!/usr/bin/env python2

from config.user import *
from config.enums import *
import math

# ===================== fixed  ================================

def x_to_d(x):
	return max((3.535 * x) - .037, x * TIME)

def BACKWARDS_X(dist):
	return (-abs(dist) + .019) / -2.972

RAD_ANGLE = DEG_ANGLE * math.pi / 180

# taken from ROS mini_max tutorial
TWIST_NUM = 20
RATE = 5.0
TIME = TWIST_NUM / RATE
TWIST_Z = 1.8/TIME		

TWIST_X = max(min(TWIST_X, .3), .1)					# ensure bounds
MIN_DIST = x_to_d(TWIST_X) * (1 + MIN_DIST_BUFFER)	# data collected by hand and fitted
													# give a 10 percent error buffer

# we don't to correct more than a quarter of the given travel distance
CHECK_MAX_COUNT = min(1, round((MIN_DIST / 4) / x_to_d(TWIST_X / CHECK_SCALE)))

CLOSE_INC = x_to_d(TWIST_X) * .05				# backwards distance we increment when too close
TOO_CLOSE = x_to_d(TWIST_X) * TOO_CLOSE_BUFFER	# want to maintain a ten percent buffer
CLOSE_MAX_COUNT = TOO_CLOSE / CLOSE_INC			# max number of backwards we should do


CENTER_INC = .1								# small value that is consistent
CENTER_TURN_DIST = .08						#
CENTER_FORWARD = (.18 + .037) / 3.535		# when we turn, we do nudge backwards a little bit
# HACK
CENTER_FORWARD = 0

COLLISION_X = BACKWARDS_X(MIN_DIST)/10		# just reset movement, but not all the way

RIGHT_SCALE = .94							# for some reason right turn > left turn
LEFT_SCALE = 1.005
