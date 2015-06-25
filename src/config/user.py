#!/usr/bin/env python2

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


