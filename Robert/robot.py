from sr.robot3 import *
import time, pathfinder
from movement import MovementMaster
from grabber import Arduino, Communicate
from position_new import Position
import os

RobotInfo_Enable = False
Arduino_Enable = True
Instructions_Enable = False
ServoBoard_Enable = False

PathFinder_Enable = False
Grabber_Enable = False
PositionFinder_Enable = True
RouteMovement_Enable = False





if Arduino_Enable:
	arduino = Arduino()
	RUGGEDUINO_ID = arduino.Setup()
	R = Robot(auto_start=True,verbose=True, ignored_ruggeduinos= [RUGGEDUINO_ID])
	com = Communicate(arduino.Start(R, RUGGEDUINO_ID))
else:
	R = Robot(auto_start=True,verbose=True)


position_finder = Position(R)
movement = MovementMaster(R)


if ServoBoard_Enable:
	R.servo_board.servos[0].position = 0
	R.servo_board.servos[1].position = 0


if RobotInfo_Enable:
	print('Starting...\n[INFO] Starting Zone ' + str(R.zone) + '\n[INFO] Arena ' + str(R.arena) + '\n[INFO] Mode ' + str(R.mode)+'\n[INFO] Robot arm radius is ' + str(movement.arm_radius)+'\n[INFO] Wheel circumference is '+str(movement.wheel_circumference))



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
		position = position_finder.try_untill_find()
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
for i in range(4):
	R.camera.save(os.path.join(R.usbkey, "initial-view" + str(i) + ".png"))
print('Finished booting press start')
'''Start button pressed run code below after start'''
while True:
	R.wait_start() 

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
		pos_get()


	if RouteMovement_Enable:
		for i in route:
			if len(i) < 2:
				print('[WARN] Invalid instruction on line ' + str(route.index(i)+1) + ' -- skipping')
				continue
			if i[0] == 'forwards':
				if len(i) == 3:
					if i[2] == 'Plough\n':
						front = [2, 0]
					elif i[2] == 'Empty\n':
						front = [1, 2]
					else:
						front = [0,1]
				else:
					front = [0,1]
				movement.forwards(float(i[1]), front)# [2,0]
			elif i[0] == 'beep':
				R.power_board.piezo.buzz(float(i[1]), Note.D6)
			elif i[0] == 'turn':
				movement.rotate(float(i[1]), 0.4)
			elif i[0] == 'sleep':
				time.sleep(float(i[1]))
			elif i[0] == 'grab' and Grabber_Enable:
				com.Grab()
			elif i[0]=='//':
				print('[WARN] Commented instruction on line ' + str(route.index(i)+1) + ' -- skipping')
			#R.wait_start()
	os.system("sync")