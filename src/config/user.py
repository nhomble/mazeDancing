#!/usr/bin/env python2

# =================== toggle functionality ==========================

# set functionality to true/false
DO_REDUCE_ANGLE		= False						# trim LaserScan data to [-DEG_ANGLE, DEG_ANGLE]
DO_CENTER			= False						# when near a wall, try to make the left and right
												# side of robot equidistant from wall
DO_GET_EDGE			= False						# attempt to detect edges and manever past them
DO_NOT_GET_CLOSE	= False						# maintain distance from wall

# =================== user defined ========================

# LaserScan detection
MIDDLE_RANGE = (-2,2)		# choose the theta range to count points as center
DEG_ANGLE = 10				# trim the sides of the LaserScan

# movement primitive
TWIST_X = .20				# hardcoded linear movement speed
MIN_DIST_BUFFER = .1		# after we calculate the approx. distance we move f(TWIST_X)
							# we add cushion to that number f(TWIST_X) * (1 + BUFFER)

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
CHECK_SCALE = 10		# when we are creeping along a detected edge we move by TWIST_X / CHECK_SCALE
CENTER_MAX_COUNT = 3	# we don't want to adjust more than 3 times
TOO_CLOSE_BUFFER = .1	# if the robot distance < TOO_CLOSE_BUFFER * MIN_DIST
