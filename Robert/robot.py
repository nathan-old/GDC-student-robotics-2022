from sr.robot3 import *
import time, pathfinder
from movement import MovementMaster
import position


R = Robot(auto_start=True,verbose=True)
''' locations of all relevant things such as cans and the centeral platform '''
can_locations = [[2.875, 0.0335], [2.875, 1.14], [2.475, 1.875], [1.625, 1.625], [3.28, 1.875], [1.875, 2.475], [1.875, 3.28], [1.14, 2.875], [0.0335, 2.875], [2.875, 5.7165], [2.875, 4.61], [3.275, 3.875], [4.125, 4.125], [2.47, 3.875], [3.875, 3.275], [3.875, 2.47], [4.61, 2.875], [5.7165, 2.875], [4.125, 1.625], [1.625, 4.125], [2.3, 2.575], [2.575, 2.3], [3.45, 2.575], [3.175, 2.3], [3.45, 3.175], [3.175, 3.45], [2.3, 3.175], [2.575, 3.45]]
obstacles = []

movement = MovementMaster(R)

print('Starting...\n[INFO] Starting Zone ' + str(R.zone) + '\n[INFO] Arena ' + str(R.arena) + '\n[INFO] Mode ' + str(R.mode)+'\n[INFO] Robot arm radius is ' + str(movement.arm_radius)+'\n[INFO] Wheel circumference is '+str(movement.circumference))

with open('route.txt', 'r') as route_file:
	route = route_file.readlines()
	for i in range(len(route)):
		route[i] = route[i].split(', ')
route_file.close()

def pos_get():
	try:
		pos = position.get_position(R)
		if pos != None:
			print('[INFO] You are in position ('+str(pos[0][0])+','+str(pos[0][1])+') at an angle of '+str(pos[1]))
			return pos
		else:
			print('[WARN] Cannot find a position')
			return None
	except:
		print('[ERROR] - Unable to get position')
		return None

print('Finished booting press start')
'''Start button pressed run code below after start'''
R.wait_start() 
for i in range(3):
	position = pos_get()
	if position != None:
		bearing = position[1]
		tolerance = 5
		if 90- tolerance > bearing or bearing > 90 + tolerance:
			movement.rotate(float(bearing-90), 0.3)
	else:
		print('[WARN] cannot find valid pos ')
		position = pos_get()
		for i in range(4):
			if position == None:
				movement.rotate(10, 0.3)
				print('[WARN] cannot find position -- attempt ' + str(i))
				position = pos_get()


position = pos_get()
for i in range(4):
	if position == None:
		print('[WARN] cannot find position -- attempt ' + str(i))
		position = pos_get()
if position != None:
	position = position[0]
	route = pathfinder.PathFind(position,can_locations, obstacles)
else:
	print('[WARN] cannot get pathfinding to work - insufficient data')

for i in route:
	if len(i) < 2:
		print('[WARN] Invalid instruction on line ' + str(route.index(i)+1) + ' -- skipping')
		continue
	if i[0] == 'forwards':
		movement.forwards(float(i[1]), 0.3)
	elif i[0] == 'beep':
		R.power_board.piezo.buzz(float(i[1]), Note.D6)
	elif i[0] == 'turn':
		movement.rotate(float(i[1]), 0.3)
	elif i[0]=='//':
		print('[WARN] Commented instruction on line ' + str(route.index(i)+1) + ' -- skipping')
for i in range(3):
	R.power_board.piezo.buzz(0.5, Note.D6)
	time.sleep(1)