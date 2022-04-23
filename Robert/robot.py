from sr.robot3 import *
import time, pathfinder
from movement import MovementMaster, RouteCommands
from grabber import Arduino, Communicate
from position import Position

RobotInfo_Enable = True
Instructions_Enable = True
PathFinder_Enable = False
Grabber_Enable = False
Set_Bearing_Enable = True


# setup all modules
arduino = Arduino()
RUGGEDUINO_ID = arduino.Setup()
R = Robot(auto_start=True,verbose=True, ignored_ruggeduinos= [RUGGEDUINO_ID])
com = Communicate(arduino.Start(R, RUGGEDUINO_ID))
position_finder = Position(R)
movement = MovementMaster(R)
routecommands = RouteCommands(R, movement, Grabber_Enable, com)

R.servo_board.servos[0].position = 0 #To give power output
R.camera.see() #start camera stream now(so its quicker to access mid-game)
print('Starting...\n[INFO] Starting Zone ' + str(R.zone) + '\n[INFO] Arena ' + str(R.arena) + '\n[INFO] Mode ' + str(R.mode)+'\n[INFO] Robot arm radius is ' + str(movement.arm_radius)+'\n[INFO] Wheel circumference is '+str(movement.wheel_circumference))



if Instructions_Enable:
	with open('route.txt', 'r') as route_file:
		route = route_file.readlines()
		for i in range(len(route)):
			route[i] = route[i].split(', ')
	route_file.close()
	if int(R.zone) == 0:
		start_beraing = 105
	elif int(R.zone) == 1:
		start_beraing = 195
	elif int(R.zone) == 2:
		start_beraing = 285
	elif int(R.zone) == 3:
		start_beraing = 15
else:
	route = []


''' locations of all relevant things such as cans and the centeral platform '''
can_locations = [[0.0335,2.875],[1.135, 2.875],[1.6,1.6],[1.6,4.15],[1.875,2.475],[1.875,3.275],[2.475,1.875],[2.475,3.875],[2.875,0.0335],[2.875,1.135],[2.875,4.615],[2.875,5.7165],[3.275,1.875],[3.275,3.875],[3.875,3.275],[3.875,2.475],[4.15,1.6],[4.15,4.15],[4.615,2.875],[5.7165,2.875]]
obstacles = [[2.275, 2.275], [2.275, 2.32], [2.275, 2.365], [2.275, 2.41], [2.275, 2.455], [2.275, 2.5], [2.275, 2.545], [2.275, 2.59], [2.275, 2.635], [2.275, 2.68], [2.275, 2.725], [2.275, 2.77], [2.275, 2.815], [2.275, 2.86], [2.275, 2.905], [2.275, 2.95], [2.275, 2.995], [2.275, 3.04], [2.275, 3.085], [2.275, 3.13], [2.275, 3.175], [2.275, 3.22], [2.275, 3.265], [2.275, 3.31], [2.275, 3.355], [2.275, 3.4], [2.275, 3.445], [2.275, 3.49], [2.32, 2.275], [2.32, 2.32], [2.32, 2.365], [2.32, 2.41], [2.32, 2.455], [2.32, 2.5], [2.32, 2.545], [2.32, 2.59], [2.32, 2.635], [2.32, 2.68], [2.32, 2.725], [2.32, 2.77], [2.32, 2.815], [2.32, 2.86], [2.32, 2.905], [2.32, 2.95], [2.32, 2.995], [2.32, 3.04], [2.32, 3.085], [2.32, 3.13], [2.32, 3.175], [2.32, 3.22], [2.32, 3.265], [2.32, 3.31], [2.32, 3.355], [2.32, 3.4], [2.32, 3.445], [2.32, 3.49], [2.365, 2.275], [2.365, 2.32], [2.365, 2.365], [2.365, 2.41], [2.365, 2.455], [2.365, 2.5], [2.365, 2.545], [2.365, 2.59], [2.365, 2.635], [2.365, 2.68], [2.365, 2.725], [2.365, 2.77], [2.365, 2.815], [2.365, 2.86], [2.365, 2.905], [2.365, 2.95], [2.365, 2.995], [2.365, 3.04], [2.365, 3.085], [2.365, 3.13], [2.365, 3.175], [2.365, 3.22], [2.365, 3.265], [2.365, 3.31], [2.365, 3.355], [2.365, 3.4], [2.365, 3.445], [2.365, 3.49], [2.41, 2.275], [2.41, 2.32], [2.41, 2.365], [2.41, 2.41], [2.41, 2.455], [2.41, 2.5], [2.41, 2.545], [2.41, 2.59], [2.41, 2.635], [2.41, 2.68], [2.41, 2.725], [2.41, 2.77], [2.41, 2.815], [2.41, 2.86], [2.41, 2.905], [2.41, 2.95], [2.41, 2.995], [2.41, 3.04], [2.41, 3.085], [2.41, 3.13], [2.41, 3.175], [2.41, 3.22], [2.41, 3.265], [2.41, 3.31], [2.41, 3.355], [2.41, 3.4], [2.41, 3.445], [2.41, 3.49], [2.455, 2.275], [2.455, 2.32], [2.455, 2.365], [2.455, 2.41], [2.455, 2.455], [2.455, 2.5], [2.455, 2.545], [2.455, 2.59], [2.455, 2.635], [2.455, 2.68], [2.455, 2.725], [2.455, 2.77], [2.455, 2.815], [2.455, 2.86], [2.455, 2.905], [2.455, 2.95], [2.455, 2.995], [2.455, 3.04], [2.455, 3.085], [2.455, 3.13], [2.455, 3.175], [2.455, 3.22], [2.455, 3.265], [2.455, 3.31], [2.455, 3.355], [2.455, 3.4], [2.455, 3.445], [2.455, 3.49], [2.5, 2.275], [2.5, 2.32], [2.5, 2.365], [2.5, 2.41], [2.5, 2.455], [2.5, 2.5], [2.5, 2.545], [2.5, 2.59], [2.5, 2.635], [2.5, 2.68], [2.5, 2.725], [2.5, 2.77], [2.5, 2.815], [2.5, 2.86], [2.5, 2.905], [2.5, 2.95], [2.5, 2.995], [2.5, 3.04], [2.5, 3.085], [2.5, 3.13], [2.5, 3.175], [2.5, 3.22], [2.5, 3.265], [2.5, 3.31], [2.5, 3.355], [2.5, 3.4], [2.5, 3.445], [2.5, 3.49], [2.545, 2.275], [2.545, 2.32], [2.545, 2.365], [2.545, 2.41], [2.545, 2.455], [2.545, 2.5], [2.545, 2.545], [2.545, 2.59], [2.545, 2.635], [2.545, 2.68], [2.545, 2.725], [2.545, 2.77], [2.545, 2.815], [2.545, 2.86], [2.545, 2.905], [2.545, 2.95], [2.545, 2.995], [2.545, 3.04], [2.545, 3.085], [2.545, 3.13], [2.545, 3.175], [2.545, 3.22], [2.545, 3.265], [2.545, 3.31], [2.545, 3.355], [2.545, 3.4], [2.545, 3.445], [2.545, 3.49], [2.59, 2.275], [2.59, 2.32], [2.59, 2.365], [2.59, 2.41], [2.59, 2.455], [2.59, 2.5], [2.59, 2.545], [2.59, 2.59], [2.59, 2.635], [2.59, 2.68], [2.59, 2.725], [2.59, 2.77], [2.59, 2.815], [2.59, 2.86], [2.59, 2.905], [2.59, 2.95], [2.59, 2.995], [2.59, 3.04], [2.59, 3.085], [2.59, 3.13], [2.59, 3.175], [2.59, 3.22], [2.59, 3.265], [2.59, 3.31], [2.59, 3.355], [2.59, 3.4], [2.59, 3.445], [2.59, 3.49], [2.635, 2.275], [2.635, 2.32], [2.635, 2.365], [2.635, 2.41], [2.635, 2.455], [2.635, 2.5], [2.635, 2.545], [2.635, 2.59], [2.635, 2.635], [2.635, 2.68], [2.635, 2.725], [2.635, 2.77], [2.635, 2.815], [2.635, 2.86], [2.635, 2.905], [2.635, 2.95], [2.635, 2.995], [2.635, 3.04], [2.635, 3.085], [2.635, 3.13], [2.635, 3.175], [2.635, 3.22], [2.635, 3.265], [2.635, 3.31], [2.635, 3.355], [2.635, 3.4], [2.635, 3.445], [2.635, 3.49], [2.68, 2.275], [2.68, 2.32], [2.68, 2.365], [2.68, 2.41], [2.68, 2.455], [2.68, 2.5], [2.68, 2.545], [2.68, 2.59], [2.68, 2.635], [2.68, 2.68], [2.68, 2.725], [2.68, 2.77], [2.68, 2.815], [2.68, 2.86], [2.68, 2.905], [2.68, 2.95], [2.68, 2.995], [2.68, 3.04], [2.68, 3.085], [2.68, 3.13], [2.68, 3.175], [2.68, 3.22], [2.68, 3.265], [2.68, 3.31], [2.68, 3.355], [2.68, 3.4], [2.68, 3.445], [2.68, 3.49], [2.725, 2.275], [2.725, 2.32], [2.725, 2.365], [2.725, 2.41], [2.725, 2.455], [2.725, 2.5], [2.725, 2.545], [2.725, 2.59], [2.725, 2.635], [2.725, 2.68], [2.725, 2.725], [2.725, 2.77], [2.725, 2.815], [2.725, 2.86], [2.725, 2.905], [2.725, 2.95], [2.725, 2.995], [2.725, 3.04], [2.725, 3.085], [2.725, 3.13], [2.725, 3.175], [2.725, 3.22], [2.725, 3.265], [2.725, 3.31], [2.725, 3.355], [2.725, 3.4], [2.725, 3.445], [2.725, 3.49], [2.77, 2.275], [2.77, 2.32], [2.77, 2.365], [2.77, 2.41], [2.77, 2.455], [2.77, 2.5], [2.77, 2.545], [2.77, 2.59], [2.77, 2.635], [2.77, 2.68], [2.77, 2.725], [2.77, 2.77], [2.77, 2.815], [2.77, 2.86], [2.77, 2.905], [2.77, 2.95], [2.77, 2.995], [2.77, 3.04], [2.77, 3.085], [2.77, 3.13], [2.77, 3.175], [2.77, 3.22], [2.77, 3.265], [2.77, 3.31], [2.77, 3.355], [2.77, 3.4], [2.77, 3.445], [2.77, 3.49], [2.815, 2.275], [2.815, 2.32], [2.815, 2.365], [2.815, 2.41], [2.815, 2.455], [2.815, 2.5], [2.815, 2.545], [2.815, 2.59], [2.815, 2.635], [2.815, 2.68], [2.815, 2.725], [2.815, 2.77], [2.815, 2.815], [2.815, 2.86], [2.815, 2.905], [2.815, 2.95], [2.815, 2.995], [2.815, 3.04], [2.815, 3.085], [2.815, 3.13], [2.815, 3.175], [2.815, 3.22], [2.815, 3.265], [2.815, 3.31], [2.815, 3.355], [2.815, 3.4], [2.815, 3.445], [2.815, 3.49], [2.86, 2.275], [2.86, 2.32], [2.86, 2.365], [2.86, 2.41], [2.86, 2.455], [2.86, 2.5], [2.86, 2.545], [2.86, 2.59], [2.86, 2.635], [2.86, 2.68], [2.86, 2.725], [2.86, 2.77], [2.86, 2.815], [2.86, 2.86], [2.86, 2.905], [2.86, 2.95], [2.86, 2.995], [2.86, 3.04], [2.86, 3.085], [2.86, 3.13], [2.86, 3.175], [2.86, 3.22], [2.86, 3.265], [2.86, 3.31], [2.86, 3.355], [2.86, 3.4], [2.86, 3.445], [2.86, 3.49], [2.905, 2.275], [2.905, 2.32], [2.905, 2.365], [2.905, 2.41], [2.905, 2.455], [2.905, 2.5], [2.905, 2.545], [2.905, 2.59], [2.905, 2.635], [2.905, 2.68], [2.905, 2.725], [2.905, 2.77], [2.905, 2.815], [2.905, 2.86], [2.905, 2.905], [2.905, 2.95], [2.905, 2.995], [2.905, 3.04], [2.905, 3.085], [2.905, 3.13], [2.905, 3.175], [2.905, 3.22], [2.905, 3.265], [2.905, 3.31], [2.905, 3.355], [2.905, 3.4], [2.905, 3.445], [2.905, 3.49], [2.95, 2.275], [2.95, 2.32], [2.95, 2.365], [2.95, 2.41], [2.95, 2.455], [2.95, 2.5], [2.95, 2.545], [2.95, 2.59], [2.95, 2.635], [2.95, 2.68], [2.95, 2.725], [2.95, 2.77], [2.95, 2.815], [2.95, 2.86], [2.95, 2.905], [2.95, 2.95], [2.95, 2.995], [2.95, 3.04], [2.95, 3.085], [2.95, 3.13], [2.95, 3.175], [2.95, 3.22], [2.95, 3.265], [2.95, 3.31], [2.95, 3.355], [2.95, 3.4], [2.95, 3.445], [2.95, 3.49], [2.995, 2.275], [2.995, 2.32], [2.995, 2.365], [2.995, 2.41], [2.995, 2.455], [2.995, 2.5], [2.995, 2.545], [2.995, 2.59], [2.995, 2.635], [2.995, 2.68], [2.995, 2.725], [2.995, 2.77], [2.995, 2.815], [2.995, 2.86], [2.995, 2.905], [2.995, 2.95], [2.995, 2.995], [2.995, 3.04], [2.995, 3.085], [2.995, 3.13], [2.995, 3.175], [2.995, 3.22], [2.995, 3.265], [2.995, 3.31], [2.995, 3.355], [2.995, 3.4], [2.995, 3.445], [2.995, 3.49], [3.04, 2.275], [3.04, 2.32], [3.04, 2.365], [3.04, 2.41], [3.04, 2.455], [3.04, 2.5], [3.04, 2.545], [3.04, 2.59], [3.04, 2.635], [3.04, 2.68], [3.04, 2.725], [3.04, 2.77], [3.04, 2.815], [3.04, 2.86], [3.04, 2.905], [3.04, 2.95], [3.04, 2.995], [3.04, 3.04], [3.04, 3.085], [3.04, 3.13], [3.04, 3.175], [3.04, 3.22], [3.04, 3.265], [3.04, 3.31], [3.04, 3.355], [3.04, 3.4], [3.04, 3.445], [3.04, 3.49], [3.085, 2.275], [3.085, 2.32], [3.085, 2.365], [3.085, 2.41], [3.085, 2.455], [3.085, 2.5], [3.085, 2.545], [3.085, 2.59], [3.085, 2.635], [3.085, 2.68], [3.085, 2.725], [3.085, 2.77], [3.085, 2.815], [3.085, 2.86], [3.085, 2.905], [3.085, 2.95], [3.085, 2.995], [3.085, 3.04], [3.085, 3.085], [3.085, 3.13], [3.085, 3.175], [3.085, 3.22], [3.085, 3.265], [3.085, 3.31], [3.085, 3.355], [3.085, 3.4], [3.085, 3.445], [3.085, 3.49], [3.13, 2.275], [3.13, 2.32], [3.13, 2.365], [3.13, 2.41], [3.13, 2.455], [3.13, 2.5], [3.13, 2.545], [3.13, 2.59], [3.13, 2.635], [3.13, 2.68], [3.13, 2.725], [3.13, 2.77], [3.13, 2.815], [3.13, 2.86], [3.13, 2.905], [3.13, 2.95], [3.13, 2.995], [3.13, 3.04], [3.13, 3.085], [3.13, 3.13], [3.13, 3.175], [3.13, 3.22], [3.13, 3.265], [3.13, 3.31], [3.13, 3.355], [3.13, 3.4], [3.13, 3.445], [3.13, 3.49], [3.175, 2.275], [3.175, 2.32], [3.175, 2.365], [3.175, 2.41], [3.175, 2.455], [3.175, 2.5], [3.175, 2.545], [3.175, 2.59], [3.175, 2.635], [3.175, 2.68], [3.175, 2.725], [3.175, 2.77], [3.175, 2.815], [3.175, 2.86], [3.175, 2.905], [3.175, 2.95], [3.175, 2.995], [3.175, 3.04], [3.175, 3.085], [3.175, 3.13], [3.175, 3.175], [3.175, 3.22], [3.175, 3.265], [3.175, 3.31], [3.175, 3.355], [3.175, 3.4], [3.175, 3.445], [3.175, 3.49], [3.22, 2.275], [3.22, 2.32], [3.22, 2.365], [3.22, 2.41], [3.22, 2.455], [3.22, 2.5], [3.22, 2.545], [3.22, 2.59], [3.22, 2.635], [3.22, 2.68], [3.22, 2.725], [3.22, 2.77], [3.22, 2.815], [3.22, 2.86], [3.22, 2.905], [3.22, 2.95], [3.22, 2.995], [3.22, 3.04], [3.22, 3.085], [3.22, 3.13], [3.22, 3.175], [3.22, 3.22], [3.22, 3.265], [3.22, 3.31], [3.22, 3.355], [3.22, 3.4], [3.22, 3.445], [3.22, 3.49], [3.265, 2.275], [3.265, 2.32], [3.265, 2.365], [3.265, 2.41], [3.265, 2.455], [3.265, 2.5], [3.265, 2.545], [3.265, 2.59], [3.265, 2.635], [3.265, 2.68], [3.265, 2.725], [3.265, 2.77], [3.265, 2.815], [3.265, 2.86], [3.265, 2.905], [3.265, 2.95], [3.265, 2.995], [3.265, 3.04], [3.265, 3.085], [3.265, 3.13], [3.265, 3.175], [3.265, 3.22], [3.265, 3.265], [3.265, 3.31], [3.265, 3.355], [3.265, 3.4], [3.265, 3.445], [3.265, 3.49], [3.31, 2.275], [3.31, 2.32], [3.31, 2.365], [3.31, 2.41], [3.31, 2.455], [3.31, 2.5], [3.31, 2.545], [3.31, 2.59], [3.31, 2.635], [3.31, 2.68], [3.31, 2.725], [3.31, 2.77], [3.31, 2.815], [3.31, 2.86], [3.31, 2.905], [3.31, 2.95], [3.31, 2.995], [3.31, 3.04], [3.31, 3.085], [3.31, 3.13], [3.31, 3.175], [3.31, 3.22], [3.31, 3.265], [3.31, 3.31], [3.31, 3.355], [3.31, 3.4], [3.31, 3.445], [3.31, 3.49], [3.355, 2.275], [3.355, 2.32], [3.355, 2.365], [3.355, 2.41], [3.355, 2.455], [3.355, 2.5], [3.355, 2.545], [3.355, 2.59], [3.355, 2.635], [3.355, 2.68], [3.355, 2.725], [3.355, 2.77], [3.355, 2.815], [3.355, 2.86], [3.355, 2.905], [3.355, 2.95], [3.355, 2.995], [3.355, 3.04], [3.355, 3.085], [3.355, 3.13], [3.355, 3.175], [3.355, 3.22], [3.355, 3.265], [3.355, 3.31], [3.355, 3.355], [3.355, 3.4], [3.355, 3.445], [3.355, 3.49], [3.4, 2.275], [3.4, 2.32], [3.4, 2.365], [3.4, 2.41], [3.4, 2.455], [3.4, 2.5], [3.4, 2.545], [3.4, 2.59], [3.4, 2.635], [3.4, 2.68], [3.4, 2.725], [3.4, 2.77], [3.4, 2.815], [3.4, 2.86], [3.4, 2.905], [3.4, 2.95], [3.4, 2.995], [3.4, 3.04], [3.4, 3.085], [3.4, 3.13], [3.4, 3.175], [3.4, 3.22], [3.4, 3.265], [3.4, 3.31], [3.4, 3.355], [3.4, 3.4], [3.4, 3.445], [3.4, 3.49], [3.445, 2.275], [3.445, 2.32], [3.445, 2.365], [3.445, 2.41], [3.445, 2.455], [3.445, 2.5], [3.445, 2.545], [3.445, 2.59], [3.445, 2.635], [3.445, 2.68], [3.445, 2.725], [3.445, 2.77], [3.445, 2.815], [3.445, 2.86], [3.445, 2.905], [3.445, 2.95], [3.445, 2.995], [3.445, 3.04], [3.445, 3.085], [3.445, 3.13], [3.445, 3.175], [3.445, 3.22], [3.445, 3.265], [3.445, 3.31], [3.445, 3.355], [3.445, 3.4], [3.445, 3.445], [3.445, 3.49], [3.49, 2.275], [3.49, 2.32], [3.49, 2.365], [3.49, 2.41], [3.49, 2.455], [3.49, 2.5], [3.49, 2.545], [3.49, 2.59], [3.49, 2.635], [3.49, 2.68], [3.49, 2.725], [3.49, 2.77], [3.49, 2.815], [3.49, 2.86], [3.49, 2.905], [3.49, 2.95], [3.49, 2.995], [3.49, 3.04], [3.49, 3.085], [3.49, 3.13], [3.49, 3.175], [3.49, 3.22], [3.49, 3.265], [3.49, 3.31], [3.49, 3.355], [3.49, 3.4], [3.49, 3.445], [3.49, 3.49]]

def pos_get():
	position = position_finder.try_untill_find()
	if position != None:
		print('[INFO] You are in position ('+str(round(position[0][0],2))+','+str(round(position[0][1],2))+') at an angle of '+str(round(position[1],2)))
		return position
	else:
		print('[WARN] Cannot find a position')
		return None

def set_bearing(bearing, tolerance = 2, tries = 3):
	for i in range(tries):
		position = pos_get()
		turn_angle = position[1] - bearing
		if turn_angle > tolerance:
			movement.rotate(turn_angle, 0.3)
print('Finished booting press start')
'''Start button pressed run code below after start'''
while True:
	R.wait_start()
	if Set_Bearing_Enable:
		set_bearing(start_beraing)
	if Grabber_Enable:
		com.Grab_No_Sleep()
		# com.Grab_No_Sleep()

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

		if position != None:
			place = position[0]
			route_found = pathfinder.PathFind(place, can_locations, obstacles)
			route_found.append(['grab', '0'])
			routecommands.follow(route_found)
			print('[INFO] found a route')
		else:
			print('[WARN] cannot get pathfinding to work - insufficient data')

	if Instructions_Enable:
		routecommands.follow(route)