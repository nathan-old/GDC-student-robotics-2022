from queue import PriorityQueue


class Spot:
	def __init__(self, row, col):
		self.row = row
		self.col = col
		self.type = None
		self.neighbors = []
		self.width = 100
		self.total_rows = 100

	def get_pos(self):
		return self.row, self.col

	def is_closed(self):
		return self.type == 'closed'

	def is_open(self):
		return self.type == 'open'

	def is_barrier(self):
		return self.type == 'barrier'

	def make_closed(self):
		self.type = 'closed'

	def make_open(self):
		self.type = 'open'

	def make_barrier(self):
		self.type = 'barrier'


	def update_neighbors(self, grid):
		self.neighbors = []
		if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT
			self.neighbors.append(grid[self.row][self.col + 1])

		if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN
			self.neighbors.append(grid[self.row + 1][self.col])

		if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP
			self.neighbors.append(grid[self.row - 1][self.col])	

		if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT
			self.neighbors.append(grid[self.row][self.col - 1])

def find_change(p1, p2):
	x1, y1 = p1
	x2, y2 = p2
	return abs(x1 - x2) + abs(y1 - y2)

class Route():
	def __init__(self):
		self.data = []
		self.routes = []
	def set_end(self, end):
		self.end = end
	def add_data(self, came_from, current):
		while current in came_from:
			current = came_from[current]
			self.data.insert(0,current.get_pos())
		self.data.append(self.end.get_pos())
	def add_can(self):
		self.routes.append(self.data)
		self.data = []
route = Route()

def algorithm(grid, start, end):
	count = 0
	open_set = PriorityQueue()
	open_set.put((0, count, start))
	came_from = {}
	g_score = {spot: float("inf") for row in grid for spot in row}
	g_score[start] = 0
	f_score = {spot: float("inf") for row in grid for spot in row}
	f_score[start] = find_change(start.get_pos(), end.get_pos())
	open_set_hash = {start}

	while not open_set.qsize() == 0:
		current = open_set.get()[2]
		open_set_hash.remove(current)
		
		if current == end:
			route.add_data(came_from, end)
			return True

		for neighbor in current.neighbors:
			temp_g_score = g_score[current] + 1

			if temp_g_score < g_score[neighbor]:
				came_from[neighbor] = current
				g_score[neighbor] = temp_g_score
				f_score[neighbor] = temp_g_score + find_change(neighbor.get_pos(), end.get_pos())
				if neighbor not in open_set_hash:
					count += 1
					open_set.put((f_score[neighbor], count, neighbor))
					open_set_hash.add(neighbor)
					neighbor.make_open()

		if current != start:
			current.make_closed()
	return False


def make_grid(size):
	'''make a square grid of size inputed'''
	grid = []
	for i in range(size):
		grid.append([])
		for j in range(size):
			grid[i].append(Spot(i,j))
	return grid



def get_route(start, cans, obstacles):
	'''start: current robot position
	cans: locations of nearest cans
	obstacles: any grid locations that are blocked
	Function to return the best route to take'''
	for i in cans:
		print('can - '+str(i))
		grid = make_grid(Spot(1,1).width)
		start_square = grid[start[0]][start[1]]
		for x in cans:
			if x != i:
				grid[x[0]][x[1]].make_barrier()
		for x in obstacles:
			grid[x[0]][x[1]].make_barrier()
		end = grid[i[0]][i[1]]
		route.set_end(end)
		for row in grid:
			for spot in row:
				spot.update_neighbors(grid)
		algorithm(grid, start_square, end)
		route.add_can()
	smallest = 100000000
	index = 0
	for i in range(len(route.routes)):
		if len(route.routes[i]) < smallest:

			smallest = len(route.routes[i])
			index = i
	print(route.routes[index])
	print(str(i)+ ' steps')
	return route.routes[index]




class Instructions():
	def __init__(self):
		self.direction = 'forwards'
		self.distance = 0
		self.grid_distance = 0.0575
		self.instruction_list = []
	def new_instruction(self):
		if len(self.instruction_list) == 1:
			if self.instruction_list[0][1] == 0:
				del self.instruction_list[0]
		self.instruction_list.append([self.direction,self.distance*self.grid_distance])
		self.instruction = None
		self.distance = 1
	def add_distance(self):
		self.distance += 1
	def change_direction(self, direction):
		self.direction = direction
	def convert(self):
		for i in range(len(self.instruction_list)):
			if self.instruction_list[i][0] == 'Reverse':
				self.instruction_list[i] = ['turn', 180],['forwards', self.instruction_list[i][1]], ['turn', -180]
			elif self.instruction_list[i][0] == 'Up':
				self.instruction_list[i:i+1] = ['turn', -90],['forwards', self.instruction_list[i][1]],['turn', 90]
			elif self.instruction_list[i][0] == 'Down':
				self.instruction_list[i:i+1] = ['turn', 90],['forwards', self.instruction_list[i][1]],['turn', -90]
		return self.instruction_list



def convert_instructions(route):
	''' route: Coordinate path outputted by get_route function
	Convert a given route into instructions the robot can follow'''
	instructions = Instructions()
	for i in range(len(route)):
		x1 = route[i][0]
		y1 = route[i][1]
		if i+1 != len(route):
			x2 = route[i+1][0]
			y2 = route[i+1][1]
			if x2-x1 == 0 and y2-y1 == 1:
				if instructions.direction != 'Down':
					instructions.new_instruction()
					instructions.change_direction('Down')
				else:
					instructions.add_distance()
			elif x2-x1 == 0 and y2-y1 == -1:
				if instructions.direction != 'Up':
					instructions.new_instruction()
					instructions.change_direction('Up')
				else:
					instructions.add_distance()
			elif y2-y1 == 0 and x2-x1 == 1:
				if instructions.direction != 'forwards':
					instructions.new_instruction()
					instructions.change_direction('forwards')
				else:
					instructions.add_distance()
			elif y2-y1 == 0 and x2-x1 == -1:
				if instructions.direction != 'Reverse':
					instructions.new_instruction()
					instructions.change_direction('Reverse')
				else:
					instructions.add_distance()
	instructions.new_instruction()
	return instructions.convert()
class convert_to_grid():
	def __init__(self):
		self.grid_distance = 0.0575
	def convert(self, distance):
		return int(distance/self.grid_distance)

def PathFind(position, can_locations, obstacles):
	'''
	position: Robots current position
	can_locations: Locations of all the cans
	obstacles: Everything in the way
	Function to find the best path to get to cans
	'''
	distance = []
	for i in can_locations:
		distance.append(find_change(position, i))
	locations = []
	for i in range(len(can_locations)):
		for x in range(len(can_locations[i])):
			can_locations[i][x] = convert_to_grid().convert(can_locations[i][x])
	for i in range(len(obstacles)):
		for x in range(len(obstacles[i])):
			obstacles[i][x] = convert_to_grid().convert(obstacles[i][x])
	for i in range(len(position)):
		position[i] = convert_to_grid().convert(position[i])
	for i in range(4):
		locations.append(can_locations[distance.index(sorted(distance)[i])])
	return convert_instructions(get_route(position, locations, obstacles))