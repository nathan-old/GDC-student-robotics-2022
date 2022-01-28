from math import *
from sr.robot3 import *
import time


R = Robot(auto_start=True, verbose=True)
# calculates the distance of the marker from the robot
def calculate_distance(theta,psi,r, num):
	h = (r*cos(radians(90-theta)))
	z = (r*cos(radians(theta)))
	x = (h*cos(radians(90-psi)))
	y = (h*sin(radians(90-psi)))
	print("x:",x,end = ', ')
	print("y:",y,end = ', ')
	print("z:",z)
	return calculate_pos(x, y, marker_list, num)

# calculates the position of the robot using the position of the marker
def calculate_pos(x_in,y_in,marker_list,marker_number):
	robot_pos = []
	for i in range(len(marker_list)):
			if marker_list[i][0] == marker_number:
				marker_pos = [marker_list[i][1],marker_list[i][2]]
	if marker_number >= 0 and marker_number <= 6:
		x = marker_pos[0]-y_in
		y = marker_pos[1]-x_in
		robot_pos = [x,y]
	elif marker_number >= 7 and marker_number <= 13:
		x = marker_pos[0]-x_in
		y = marker_pos[1]-y_in
		robot_pos = [x,y]
	elif marker_number >= 14 and marker_number <= 20:
		x = marker_pos[0]+y_in
		y = marker_pos[1]+x_in
		robot_pos = [x,y]
	elif marker_number >= 19 and marker_number <= 27:
		x = marker_pos[0]+x_in
		y = marker_pos[1]+y_in
		robot_pos = [x,y]
	return(robot_pos)

def get_markers():
	'''
	inputs from robot camera in degrees and mm
	list of markers seen by camera, in order theta, psi, r, marker number
	'''
	markers = []
	markers_input = R.camera.see()
	for marker in markers_input:
		markers.append([degrees(marker.spherical.rot_x),degrees(marker.spherical.rot_y),marker.spherical.dist*1000,marker.id])
	if len(markers) == 0:
		print('No markers, trying again in 1 s')
		time.sleep(1)
		return get_markers()
	return markers

# list of marker positions
marker_list = [[0,718.75,5750],[1,1437.5,5750],[2,2156.25,5750],[3,2875,5750],[4,3593.75,5750],[5,4312.5,5750],[6,5031.25,5750],[7,5750,5031.25],[8,5750,4312.5],[9,5750,3593.75],[10,5750,2875],[11,5750,2156.25],[12,5750,1437.5],[13,5750,718.75],[14,5031.25,0],[15,4312.5,0],[16,3593.75,0],[17,2875,0],[18,2156.25,0],[19,1437.5,0],[20,718.75,0],[21,0,718.75],[22,0,1437.5],[23,0,2156.25],[24,0,2875],[25,0,3593.75],[26,0,4312.5],[27,0,5031.25]]


R.wait_start()#start command

for i in range(6):
        print(get_markers())
        R.power_board.piezo.buzz(0.2, 500)
        time.sleep(3)
R.power_board.piezo.buzz(0.2, 350)
time.sleep(0.2)
R.power_board.piezo.buzz(0.2, 350)
time.sleep(0.2)
R.power_board.piezo.buzz(0.2, 350)
time.sleep(0.2)