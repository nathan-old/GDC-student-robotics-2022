from sr.robot3 import *
import time, pathfinder
from movement import MovementMaster
from grabber import Arduino, Communicate
from position import PositionFinder


RobotInfo_Enable = True
Arduino_Enable = False
Instructions_Enable = True
ServoBoard_Enable = False

PathFinder_Enable = False
Grabber_Enable = False
PositionFinder_Enable = False
RouteMovement_Enable = True




if Arduino_Enable:
	arduino = Arduino()
	RUGGEDUINO_ID = arduino.Setup()
	R = Robot(auto_start=True,verbose=True, ignored_ruggeduinos= [RUGGEDUINO_ID])
	com = Communicate(arduino.Start(R, RUGGEDUINO_ID))
else:
	R = Robot(auto_start=True,verbose=True)


position_finder = PositionFinder(R)
movement = MovementMaster(R)


if ServoBoard_Enable:
	R.servo_board.servos[0].position = 0
	R.servo_board.servos[1].position = 0


if RobotInfo_Enable:
	print('Starting...\n[INFO] Starting Zone ' + str(R.zone) + '\n[INFO] Arena ' + str(R.arena) + '\n[INFO] Mode ' + str(R.mode)+'\n[INFO] Robot arm radius is ' + str(movement.arm_radius)+'\n[INFO] Wheel circumference is '+str(movement.circumference))


if Grabber_Enable:
	for i in range(2):
		com.Grab()
		print('Grab')
		time.sleep(2)


if Instructions_Enable:
	with open('route.txt', 'r') as route_file:
		route = route_file.readlines()
		for i in range(len(route)):
			route[i] = route[i].split(', ')
	route_file.close()
else:
	route = []



''' locations of all relevant things such as cans and the centeral platform '''
can_locations = [[2.875, 0.0335], [2.875, 1.14], [2.475, 1.875], [1.625, 1.625], [3.28, 1.875], [1.875, 2.475], [1.875, 3.28], [1.14, 2.875], [0.0335, 2.875], [2.875, 5.7165], [2.875, 4.61], [3.275, 3.875], [4.125, 4.125], [2.47, 3.875], [3.875, 3.275], [3.875, 2.47], [4.61, 2.875], [5.7165, 2.875], [4.125, 1.625], [1.625, 4.125], [2.3, 2.575], [2.575, 2.3], [3.45, 2.575], [3.175, 2.3], [3.45, 3.175], [3.175, 3.45], [2.3, 3.175], [2.575, 3.45]]
obstacles = []


#R.servo_board.servos[1].position = 0.2
def pos_get():
	try:
		position = position_finder(R).run()
		if position != None:
			print('[INFO] You are in position ('+str(position[0][0])+','+str(position[0][1])+') at an angle of '+str(position[1]))
			return position
		else:
			print('[WARN] Cannot find a position')
			return None
	except Exception as error:
		print('[ERROR] - Unable to get position')
		print('-----------------------------------------------------------------------------------------')
		print(error)
		print('-----------------------------------------------------------------------------------------')
		return None

print('Finished booting press start')
'''Start button pressed run code below after start'''
R.wait_start() 



if Grabber_Enable:
	for i in range(5):
			com.Grab()
			time.sleep(2)


if PathFinder_Enable:
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



if PositionFinder_Enable:
	print(pos_get())


if RouteMovement_Enable:
	for i in route:
		if len(i) < 2:
			print('[WARN] Invalid instruction on line ' + str(route.index(i)+1) + ' -- skipping')
			continue
		if i[0] == 'forwards':
			movement.forwards(float(i[1]), [2,0])
		elif i[0] == 'beep':
			R.power_board.piezo.buzz(float(i[1]), Note.D6)
		elif i[0] == 'turn':
			movement.rotate(float(i[1]), 0.3)
		elif i[0] == 'sleep':
			time.sleep(float(i[1]))
		elif i[0]=='//':
			print('[WARN] Commented instruction on line ' + str(route.index(i)+1) + ' -- skipping')
	time.sleep(1)
	for i in range(3):
		R.power_board.piezo.buzz(0.5, Note.D6)
		time.sleep(1)