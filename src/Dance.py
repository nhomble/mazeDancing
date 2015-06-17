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
	for d in directions:
		clock = Language.DIR_TO_CLOCK[d]
		move.move(clock[0])
		rospy.sleep(DANCE_DELAY)
		move.move(clock[1])

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

def _translate(l, trans):
	i = 2
	ret = []
	while i < len(l) - 1:
		couple = (l[i], l[i+1])
		ret.append(trans[couple[0]][couple[1]])
		i += 2
	return ret

def _get_turns(tags):
	return _translate(tags, Language.ID_TO_CLOCK)
def _get_directions(turns):
	return _translate(turns, Language.CLOCK_TO_DIR)

def _check_is_done(tags):
	global _dirs
	global _turns
	_turns = _get_turns(tags)
	_dirs = _get_directions(_turns)
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
		_is_done = False if len(_detected_tags) < 4 or len(_detected_tags) % 2 != 1 else _check_is_done(_detected_tags)
	
#	if _is_done:
#		del _dirs[-1]

# need to do image processing to detect movements
def interpret_dance():
	_detected_turns = []
	sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, _tag_callback)
	while not _is_done and not rospy.is_shutdown():
		rospy.loginfo(_detected_tags)
		rospy.loginfo(_turns)
		rospy.loginfo(_dirs)
		rospy.Rate(DELAY).sleep()
	sub.unregister()
	return _dirs

'''
========================================================
'''
