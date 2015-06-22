#!/usr/bin/env python2

import math
import roslib; roslib.load_manifest('mazeDancing')
import rospy
from consts import *
import time
import rospy
from ar_track_alvar.msg import *

'''
SCOUT
'''

# for each movement:
#	move in that direction for a whole circle
#	then pause  to delimit the directions
def do_dance(directions, move):
	# add terminator
	directions.append(None)
	_hist = []
	deg = 0
	for d in directions:
		rospy.loginfo(_hist)
		rospy.loginfo(Direction.to_string[d])
		clock = Language.DIR_TO_CLOCK[d]
		move.move(clock[0])
		rospy.sleep(DANCE_DELAY)
		move.move(clock[1])
		rospy.sleep(DANCE_DELAY)
		_hist.append(clock[0])
		_hist.append(clock[1])
		if clock[0] == Turn.CLOCKWISE:
			deg += 90
		else:
			deg -= 90
		if clock[1] == Turn.CLOCKWISE:
			deg += 90
		else:
			deg -= 90
	while deg < 0:
		deg += 90
	while deg > 0:
		deg -= 90
		move.move(Direction.LEFT)
		

'''
=======================================================
'''

'''
WORKER
'''

_detected_tags = []
_last_tag = None
_is_done = False
_dirs = []
_turns = []
_start = False

def _translate(l, trans):
	i = 0
	ret = []
	while i < len(l) - 1:
		couple = (l[i], l[i+1])
		ret.append(trans[couple[0]][couple[1]])
		i += 1
	return ret

def _couple(l, trans):
	i = 0
	ret = []
	while 2*i < len(l):
		couple = (l[2*i],l[2*i + 1])
		ret.append(trans[couple[0]][couple[1]])
		i += 1
	return ret

def _get_turns(tags):
	return _translate(tags, Language.ID_TO_CLOCK)
def _get_directions(turns):
	# HACK
	ret = _couple(turns, Language.CLOCK_TO_DIR)
	return ret

def _check_is_done(tags):
	global _dirs
	global _turns
	_turns = _get_turns(tags)
	try:
		_dirs = _get_directions(_turns)
	except:
		pass
	return True if _dirs[-1] is None else False

def _tag_callback(data):
	global _detected_tags
	global _last_tag
	global _is_done
	global _dirs
	if len(data.markers) == 0:
		return
	_marker  = data.markers[0]
	if _marker.id < 3 or _marker.id > 7:
		return
	tag = Tag.translate_id(_marker.id)
	if _is_done:
		return
	# init
	if not _start:
		global _start
		if tag != Tag.FRONT:
			return
		else:
			_start = True
	if _last_tag is None:
		_last_tag = tag
		_detected_tags.append(tag)
	# we got a new position
	elif _last_tag != tag:
		# sometimes we skip
#		if abs(tag[0] - _last_tag[0]) == 2:
#			_detected_tags.append(Tag.translate_id((tag[0] + _last_tag[0]) / 2))
		_last_tag = tag
		_detected_tags.append(tag)

		# check end condition
		# we need at least two to combine
		_is_done = False if len(_detected_tags) < 2 else _check_is_done(_detected_tags)
	
#	if _is_done:
#		del _dirs[-1]

# need to do image processing to detect movements
def interpret_dance():
	_detected_turns = []
	sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, _tag_callback)
	while not _is_done and not rospy.is_shutdown():
		rospy.loginfo(_detected_tags)
		rospy.Rate(DELAY).sleep()
	sub.unregister()
	return _dirs

'''
========================================================
'''

def _test():
	global _is_done
	directions = [Direction.LEFT, None]
	tags = [Tag.FRONT, Tag.LEFT, Tag.BACK, Tag.LEFT, Tag.FRONT]
	_is_done = _check_is_done(tags)
	print("is done: " + str(_is_done))
	if _is_done:
		print(_dirs)
