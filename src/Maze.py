#!/usr/bin/env python2

class Cell(object):
	UNKNOWN = 1,
	OPEN = 2,
	WALL = 3

class Maze(object):
	def __init__(self, n=20):
		self.orientation = Direction.FORWARD
		self.maze = [[Cell.UNKNOWN for i in range(n)] for j in range(n)]
		self.pos = (n//2, n//2)

	# get new step
	def bot_step(self):
		new_pos = None
		if self.orientation == Direction.FORWARD:
			new_pos = (self.pos[0] - 1, self.pos[1])
		elif self.orientation == Direction.RIGHT:
			new_pos = (self.pos[0], self.pos[1] + 1])
		elif self.orientation == Direction.BACKWARD:
			new_pos = (self.pos[0] + 1, self.pos[1])
		elif self.orientation == Direction.LEFT:
			new_pos = (self.pos[0], self.pos[1] - 1)

		# update based on new point
		self._update_maze(new_pos)

	# update orientation
	def bot_turn(self, direction):
		if direction == Direction.FORWARD or direction == Direction.BACKWARD:
			raise Exception("bad direction: " + direction)
		val = 1 if direction == Direction.RIGHT else 3
		self.pos %= self.pos + val
	
	# check bounds on our new point
	def _new_pos_is_valid(self, x, y):
		maze = len(self.maze)
		if x < 0 or y < 0 or x >= maze or y >= maze:
			return False
		return True
	
	def _update_maze(self, new_pos):
		# check bounds
		if not self._new_pos_is_valid(*new_pos):
			self.maze = _expand_maze(self.maze)
			n = len(self.maze)//2
			new_pos = (new_pos[0] + n, new_pos[1] + n)

		# move
		self.pos = new_pos
		# mark internally
		self.maze[self.pos[0]][self.pos[1]] = Cell.OPEN
	
	# use _best_path but remove duplicates
	# for communication
	def best_path(self):
		pass

	# return a series of directions
	def _best_path(self):
		pass

# take nxn maze and pad to 2nx2n
def _expand_maze(maze):
	pass
