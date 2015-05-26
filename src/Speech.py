#!/usr/bin/env python2

import pexpect
import time

class Speech_Manager(object):
	def __init__(self):
		self.espeak = pexpect.spawn('espeak') 
	def speak(self, phrase):
		self.espeak.sendline(phrase)
