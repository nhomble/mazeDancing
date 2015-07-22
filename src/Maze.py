#!/usr/bin/env python2

import numpy
import IPython
from consts import *
from copy import copy, deepcopy
from a_star import *

class Maze(object):
	def __init__(self, n=10):
		self.orientation = Direction.FORWARD
		self.maze = [[Maze_Cell.UNKNOWN for i in range(n)] for j in range(n)]
		self.pos = (n//2, n//2)
		self.last_pos = None
		self.start = self.pos
	#	self.maze[self.start[0]][self.start[1]] = Maze_Cell.OPEN
		self.maze = [[(1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (2,), (2,), (2,), (2,), (1,), (1,), (1,)], [(1,), (1,), (1,), (2,), (1,), (1,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (2,), (2,), (2,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (2,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (2,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,)]]		
		self.maze2 = [[(1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (2,), (2,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (2,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (2,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (2,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (2,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,)], [(1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,), (1,)]]
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

	def get_status(self, direction):
		v = Maze_Cell.POS_FROM_OD[self.orientation][direction]
		pos = (self.pos[0] + v[0], self.pos[1] + v[1])
		return self.maze[pos[0]][pos[1]]

	def need_to_check(self, direction):
		return self.get_status(direction) == Maze_Cell.UNKNOWN
	
	def mark_ahead(self):
		v = Maze_Cell.POS_FROM_OD[self.orientation][Direction.FORWARD]
		_pos = (self.pos[0] + v[0], self.pos[1] + v[1])

		if not self._new_pos_is_valid(*_pos):
			self._expand_maze()
			n = len(self.maze)//2
			_pos = (_pos[0] + n//2, _pos[1] + n//2)
			self.start = (self.start[0] + n//2, self.start[1] + n//2)


		self.maze[_pos[0]][_pos[1]] = Maze_Cell.OPEN	

	def next_pos(self, t=Follower.RIGHT):
		self.print_maze()
		if t == Follower.RIGHT:
			if self.maze[self.pos[0]][self.pos[1] + 1] == Maze_Cell.OPEN:
				leave_dir = Direction.RIGHT
			elif self.maze[self.pos[0] - 1][self.pos[1]] == Maze_Cell.OPEN:
				leave_dir = Direction.FORWARD
			elif self.maze[self.pos[0]][self.pos[1] - 1] == Maze_Cell.OPEN:
				leave_dir = Direction.LEFT
			else:
				leave_dir = Direction.BACKWARD
			return leave_dir

	# update orientation
	def turn(self, direction):
		self.orientation = Direction.TURN[self.orientation][direction]
	
	# check bounds on our new point
	def _new_pos_is_valid(self, x, y):
		maze = len(self.maze)
		if x < 0 or y < 0 or x >= maze or y >= maze:
			return False
		return True

	def collision(self):
		self.maze[self.pos[0]][self.pos[1]] = Maze_Cell.WALL
		self.pos = self.last_pos
	
	def _update_maze(self, new_pos):
		# check bounds
		if not self._new_pos_is_valid(*new_pos):
			self._expand_maze()
			n = len(self.maze)//2
			new_pos = (new_pos[0] + n//2, new_pos[1] + n//2)
			self.start = (self.start[0] + n//2, self.start[1] + n//2)

		# move
		self.last_pos = self.pos
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
		return _best_scout_path(a_star, self.maze), _best_worker_path(a_star, self.maze)

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
		x = 0
		for row in self.maze:
			string = ""
			x += 1
			y = 0
			for ele in row:
				if x == self.pos[0] and y == self.pos[1]:
					string += "*"
				elif ele == Maze_Cell.OPEN:
					string += " "
				else:
					string += "X"
				y += 1
			print(string)
	
	def new_print_maze(self, maze):
	       
		
		#maze = self.develop_maze()
		
	        x = 0
                for row in maze:
                        string = ""
                        x += 1
                        y = 0
                        for ele in row:
                                if x == self.pos[0] and y == self.pos[1]:
                                        string += "*"
                                elif ele == Maze_Cell.OPEN:
                                        string += " "
                                else:
                                        string += "X"
                                y += 1
                        print(string)
                print(maze)


	
	#These directions are if you want to communicate specific turns for the robot to take	
	def generate_directions(self):
		if(self.maze[self.start[0]][self.start[1]] == Maze_Cell.OPEN):
			print("Ready")
		x = self.start[0]
 		y = self.start[1]
		i = "Forward"		
		
		temp_arr = self.maze
		directions_arr = []
		
		while(True):
			if(temp_arr[x-1][y] == Maze_Cell.OPEN):
				if(i == "Forward"):
					directions_arr.append("Forward")
					temp_arr[x][y] = 0
					x = x - 1
				elif(i == "Left"):
					directions_arr.append("Right")
					i = "Forward"
				elif(i == "Right"):
					directions_arr.append("Left")
					i = "Forward"
				else:
					i = "Forward"
			elif(temp_arr[x][y-1] == Maze_Cell.OPEN):
				if(i == "Forward"):
					directions_arr.append("Left")
					i = "Left"
				elif(i == "Left"):
					directions_arr.append("Forward")
					temp_arr[x][y] = 0
					y = y - 1
				else:
					i = "Forward"
			elif(temp_arr[x][y+1] == Maze_Cell.OPEN):
				if(i == "Forward"):
					directions_arr.append("Right")
					i = "Right"
				elif(i == "Right"):
					directions_arr.append("Forward")
					temp_arr[x][y] = 0
					y = y + 1
				else:
					i = "Forward"
			else:
				break
		
		proper_arr = []
		for p in directions_arr:
			if(p == "Forward"):
				proper_arr.append(Direction.FORWARD)
			elif(p == "Left"):
				proper_arr.append(Direction.LEFT)
			else:
				proper_arr.append(Direction.RIGHT)
	
		print(directions_arr)
		print(proper_arr)
		return proper_arr
	
	#This makes it easier to develop a map of the array
	def generate_path_directions(self, maze):
		if(self.maze[self.start[0]][self.start[1]] == Maze_Cell.OPEN):
			print("Ready")
		x = self.start[0]
 		y = self.start[1]
		i = 0		
		
		temp_arr = maze
		directions_arr = []
		while(True):
			if(temp_arr[x-1][y] == Maze_Cell.OPEN):
				directions_arr.append("Forward")
				temp_arr[x][y] = 0
				x = x - 1
			elif(temp_arr[x][y-1] == Maze_Cell.OPEN):
				directions_arr.append("Left")
				temp_arr[x][y] = 0
				y = y - 1
			elif(temp_arr[x][y+1] == Maze_Cell.OPEN):
				directions_arr.append("Right")
				temp_arr[x][y] = 0
				y = y + 1
			else:
				break
		print(x)
		print(y)
		proper_arr = []
                for p in directions_arr:
                        if(p == "Forward"):
                                proper_arr.append(Direction.FORWARD)
                        elif(p == "Left"):
                                proper_arr.append(Direction.LEFT)
                        else:
                                proper_arr.append(Direction.RIGHT)
		return proper_arr
	
	#Uses path_directions to create a corresponding maze
	def develop_maze(self, arr):
		orientation = Direction.FORWARD
		n = 10
                newMaze = [[Maze_Cell.UNKNOWN for i in range(n)] for j in range(n)]
                pos = (n//2, n//2)
                last_pos = None
                start = self.pos
                newMaze[self.start[0]][self.start[1]] = Maze_Cell.OPEN
		x = self.start[0]
		y = self.start[1]		
		
		print(arr)
		for p in arr:
			if(p == Direction.FORWARD):
				newMaze[x-1][y] = Maze_Cell.OPEN
				x = x - 1
			elif(p == Direction.LEFT):
				newMaze[x][y-1] = Maze_Cell.OPEN
				y = y - 1
			else:
				newMaze[x][y+1] = Maze_Cell.OPEN
				y = y + 1

		self.new_print_maze(newMaze)	
		
	def overlap_mazes(self, maze1, maze2):
		#find last open position in both mazes

		x = self.start[0]
                y = self.start[1]
				
                temp_arr = deepcopy(maze1)

                while(True):
                        if(temp_arr[x-1][y] == Maze_Cell.OPEN):  
                                temp_arr[x][y] = 0
                                x = x - 1
			elif(temp_arr[x][y-1] == Maze_Cell.OPEN):
                                temp_arr[x][y] = 0
                                y = y - 1
                        elif(temp_arr[x][y+1] == Maze_Cell.OPEN):
                                temp_arr[x][y] = 0
                                y = y + 1
                        else:
                                break

 		last_pos_x1 = x
		last_pos_y1 = y
		
		x = self.start[0]
                y = self.start[1]
                i = 0
		
                temp_arr = deepcopy(maze2)
                while(True):
                        if(temp_arr[x-1][y] == Maze_Cell.OPEN):
                                temp_arr[x][y] = 0
                                x = x - 1
                        elif(temp_arr[x][y-1] == Maze_Cell.OPEN):
                                temp_arr[x][y] = 0
                                y = y - 1
                        elif(temp_arr[x][y+1] == Maze_Cell.OPEN):
                                temp_arr[x][y] = 0
                                y = y + 1
                        else:
                                break
		
		last_pos_x2 = x
		last_pos_y2 = y
		
		str = "The positions of maze1 are ( %s , %s ) and the positions of maze2 are ( %s , %s)" % (last_pos_x1, last_pos_y1, last_pos_x2, last_pos_y2)
		print(str)
		
		#Determine how many units to shift over
		offset_x = 0
		offset_y = 0
		if(last_pos_x1 == last_pos_x2):
			offset_x = 0
		else:
			offset_x = abs(last_pos_x1 - last_pos_x2)
	
		if(last_pos_y1 == last_pos_y2):
			offset_y = 0
		else:
			offset_y = abs(last_pos_y1 - last_pos_y2)
		print(offset_x)
		print(offset_y)
		
		#Create Offset Maze		
		n = 10
                offsetMaze = [[Maze_Cell.UNKNOWN for i in range(n)] for j in range(n)]
                pos = (n//2, n//2)
		x = self.start[0]
                y = self.start[1]
		
		
                temp_arr = deepcopy(maze2)
		while(True):
                        if(temp_arr[x-1][y] == Maze_Cell.OPEN):
				offsetMaze[x + offset_x][y + offset_y] = temp_arr[x][y]
                                temp_arr[x][y] = 0
                                x = x - 1
                        elif(temp_arr[x][y-1] == Maze_Cell.OPEN):
              			offsetMaze[x + offset_x][y + offset_y] = temp_arr[x][y]
                                temp_arr[x][y] = 0
                                y = y - 1
                        elif(temp_arr[x][y+1] == Maze_Cell.OPEN):
                                offsetMaze[x + offset_x][y + offset_y] = temp_arr[x][y]
				temp_arr[x][y] = 0
                                y = y + 1
                        else:
                                break
		
      		offsetMaze[x + offset_x][y + offset_y] = temp_arr[x][y]
		self.new_print_maze(offsetMaze)
		
		#Merge maze1 and offsetMaze
		opens_arr = []
			
		for j in range(10):
			for i in range(10):
				if(offsetMaze[i][j] == Maze_Cell.OPEN):
					opens_arr.append(i)
					opens_arr.append(j)
		print(opens_arr)

		mergeMaze = deepcopy(maze1)
		
		a = 0		
		while(a < (len(opens_arr))):
			if(a+1 >= len(opens_arr)):
				break
			else:
				c = opens_arr[a]
				d = opens_arr[a+1]
				mergeMaze[c][d] = Maze_Cell.OPEN
				a = a + 2

		self.new_print_maze(mergeMaze)
	
		
def _best_scout_path(a_star, maze):
	cells = []
	cell = a_star.end
	while cell is not a_star.start:
		cells.append(cell)
		cell = cell.parent
	cells.append(cell)
	return _extract_path(cells, maze)
	
def _best_worker_path(a_star, maze):
	# HACK
	scout = _best_scout_path(a_star, maze)
	_rev = scout[::-1]
	ret = []
	for d in _rev:
		if d == Direction.RIGHT:
			ret.append(Direction.LEFT)
		elif d == Direction.LEFT:
			ret.append(Direction.RIGHT)
		elif d == Direction.FORWARD:
			ret.append(Direction.BACKWARD)
		else:
			ret.append(Direction.FORWARD)
	return ret

def _extract_path(cells, maze):
	# of these cells, which are actual nodes
	nodes = []
	for cell in cells:
		out = 0
		if Maze_Cell.OPEN == maze[cell.x-1][cell.y]:
			out += 1
		if Maze_Cell.OPEN == maze[cell.x+1][cell.y]:
			out += 1
		if Maze_Cell.OPEN == maze[cell.x][cell.y-1]:
			out += 1
		if Maze_Cell.OPEN == maze[cell.x][cell.y+1]:
			out += 1
		if out > 2:
			nodes.append(cell)
	ret = []
	last = (False, None, None)
	for cell in cells:
		# cell is how I left
		# last[1] is the node's position
		# last[2] is how I entered
		if last[0]:
			# NOTE
			# coordinates are mixed up
			leave_x = cell.y - last[1].y
			leave_y = cell.x - last[1].x
			enter_x = last[1].y - last[2].y
			enter_y = last[1].x - last[2].x
			print("{} {} : {} {}".format(enter_x, enter_y, leave_x, leave_y))

			leave_dir = Maze_Cell.CELL_TO_DIR[leave_x][leave_y]
			enter_dir = Maze_Cell.CELL_TO_DIR[enter_x][enter_y]
			direction = Maze_Cell.DIR_TO_TURN[enter_dir][leave_dir]
			ret.append(direction)
			print("{} {} {}".format(Direction.to_string[leave_dir], Direction.to_string[enter_dir], Direction.to_string[direction]))
		last = (cell in nodes, cell, last[1])

	return ret


a = Maze()
#a.new_print_maze(a.maze2)
#a.generate_directions()
#arr = a.generate_path_directions()
#a.develop_maze(arr)	
a.overlap_mazes(a.maze, a.maze2)

