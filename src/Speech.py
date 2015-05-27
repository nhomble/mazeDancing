#!/usr/bin/env python2

import pexpect
import time

# wrap pexpect
class Speech_Manager(object):
	def __init__(self):
		self.espeak = pexpect.spawn('espeak') 
	def speak(self, phrase):
		self.espeak.sendline(phrase)
