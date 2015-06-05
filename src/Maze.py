#!/usr/bin/env python2

import numpy
from consts import *

from a_star import *
class Maze(object):
	def __init__(self, n=8):
		self.orientation = Direction.FORWARD
		self.maze = [[Maze_Cell.UNKNOWN for i in range(n)] for j in range(n)]
		self.pos = (n//2, n//2)
		self.start = self.pos
		self.maze[self.start[0]][self.start[1]] = Maze_Cell.OPEN

		self.path = []

	# get new step
	def step(self):
		new_pos = None
		if self.orientation == Direction.FORWARD:
			new_pos = (self.pos[0] - 1, self.pos[1])
		elif self.orientation == Direction.RIGHT:
			new_pos = (self.pos[0], self.pos[1] + 1)
		elif self.orientation == Direction.BACKWARD:
			new_pos = (self.pos[0] + 1, self.pos[1])
		elif self.orientation == Direction.LEFT:
			new_pos = (self.pos[0], self.pos[1] - 1)

		# update based on new point
		self._update_maze(new_pos)

	# update orientation
	def turn(self, direction):
		self.orientation = Direction.TURN[self.orientation][direction]
	
	# check bounds on our new point
	def _new_pos_is_valid(self, x, y):
		maze = len(self.maze)
		if x < 0 or y < 0 or x >= maze or y >= maze:
			return False
		return True
	
	def _update_maze(self, new_pos):
		# check bounds
		if not self._new_pos_is_valid(*new_pos):
			self._expand_maze()
			n = len(self.maze)//2
			new_pos = (new_pos[0] + n//2, new_pos[1] + n//2)
			self.start = (self.start[0] + n//2, self.start[1] + n//2)

		# move
		self.pos = new_pos
		# mark internally
		self.maze[self.pos[0]][self.pos[1]] = Maze_Cell.OPEN
	
	# return best path from self.start to self.pos
	# based on self.maze
	def best_path(self):
		# A* algo
		a_star = AStar()
		a_star.init_maze(self.maze, self.start, self.pos)
		a_star.process()

		# get path from algo
		cells = []
		cell = a_star.end
		while cell is not a_star.start:
			cells.insert(0, cell)
			cell = cell.parent
		cells.insert(0, a_star.start)
		
		# translate into movements
		dirs = []
		last_dir = Direction.FORWARD
		for cell in cells:
			continue
		return None
	
	# take nxn maze and pad to 2nx2n
	def _expand_maze(self):
		n = len(self.maze)
		padding = [Maze_Cell.UNKNOWN for i in range(n//2)]
		new_maze = []
		# add first n/2 rows
		for i in range(n//2):
			new_maze.append([Maze_Cell.UNKNOWN for j in range(2*n)])
		for row in self.maze:
			new_maze.append(padding + row + padding)
		for i in range(n//2):
			new_maze.append([Maze_Cell.UNKNOWN for j in range(2*n)])
		self.maze = new_maze

	def print_maze(self):
		for row in self.maze:
			string = ""
			for ele in row:
				if ele == Maze_Cell.OPEN:
					string += " "
				else:
					string += "X"
			print(string)
