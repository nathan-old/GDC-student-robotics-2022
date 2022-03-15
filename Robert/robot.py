from sr.robot3 import *
import time
from movement import MovementMaster
#import position as pos
R = Robot(auto_start=True,verbose=True)

zone = R.zone
arena = R.arena
robot_mode = R.mode
print('Starting...\n[INFO] Starting Zone ' + str(zone) + '\n[INFO] Arena ' + str(arena) + '\n[INFO] Mode ' + str(robot_mode))

with open('route.txt', 'r') as route_file:
	route = route_file.readlines()
	for i in range(len(route)):
		route[i] = route[i].split(', ')

movement = MovementMaster(R)
def pos_get():
	try:
		position = pos.get_position(R)
		if position != None:
			print('[INFO] You are in position ('+str(position[0][0])+','+str(position[0][1])+') at an angle of '+str(position[1]))
			print(position)
		else:
			print('[WARN] Cannot find a position')
	except:
		print('[ERROR] - Unable to get position')
print('[INFO] Robot arm radius is ' + str(movement.arm_radius))
print('[INFO] Wheel circumference is '+str(movement.circumference))
print('Finished booting press start')
R.wait_start()
#pos_get()
for i in route:
	if len(i) != 2:
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
#pos_get()
for i in range(3):
	R.power_board.piezo.buzz(0.5, Note.D6)
	time.sleep(1)