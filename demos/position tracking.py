from math import *
from sr.robot3 import *
import time

R = Robot(auto_start=True, verbose=True)
wheels = [robot.motor_boards['SR0PJ1W'].motors[0],robot.motor_boards['SR0PJ1W'].motors[1],robot.motor_boards['SR0VG1M'].motors[0]]
arm_radius = 0.28 # radius of the arms of the robot/ m
speed = 0.7697666666666666666667 # speed of the motors at power = 1/ m/s
    

def rotate(angle):
	distance = angle * ((math.pi * 2 * arm_radius) / 360)
	print("Turning " + str(angle) + " degrees, " + str(distance) +" meters")
	time_ = float(distance / speed)
	for wheel in wheels:
		wheel.power = 1
	time.sleep(time_)
	for wheel in wheels:
		wheel.power = 0


def get_intersections(x0, y0, r0, x1, y1, r1):
	d=sqrt((x1-x0)**2 + (y1-y0)**2)
	if d > r0 + r1 :
		return [[-1,-1],[-1,-1]]
	if d < abs(r0-r1):
		return [[-1,-1],[-1,-1]]
	if d == 0 and r0 == r1:
		return [[-1,-1],[-1,-1]]
	else:
		a=(r0**2-r1**2+d**2)/(2*d)
		h=sqrt(r0**2-a**2)
		x2=x0+a*(x1-x0)/d   
		y2=y0+a*(y1-y0)/d   
		x3=x2+h*(y1-y0)/d     
		y3=y2-h*(x1-x0)/d 
		x4=x2-h*(y1-y0)/d
		y4=y2+h*(x1-x0)/d
		return ([[x3, y3], [x4, y4]])

def calculate_distance(theta,psi,r, num):
	h = (r*cos(radians(90-theta)))
	return h


def get_markers():
	'''
	inputs from robot camera in degrees and mm
	list of markers seen by camera, in order theta, psi, r, marker number
	'''
	markers = []
	retry_count = 0
	while len(markers != 2):
		markers_input = R.camera.see()
		for marker in markers_input:
			markers.append([degrees(marker.spherical.rot_x),degrees(marker.spherical.rot_y),marker.spherical.dist,marker.id])
		if len(markers) != 2:
			retry_count += 1
			if retry_count < 5:
				print('Not enough markers, trying again in 0.5s')
				time.sleep(0.5)
			else:
				rotate(10)
	return markers

# list of marker positions
marker_list = [[0,718.75,5750],[1,1437.5,5750],[2,2156.25,5750],[3,2875,5750],[4,3593.75,5750],[5,4312.5,5750],[6,5031.25,5750],[7,5750,5031.25],[8,5750,4312.5],[9,5750,3593.75],[10,5750,2875],[11,5750,2156.25],[12,5750,1437.5],[13,5750,718.75],[14,5031.25,0],[15,4312.5,0],[16,3593.75,0],[17,2875,0],[18,2156.25,0],[19,1437.5,0],[20,718.75,0],[21,0,718.75],[22,0,1437.5],[23,0,2156.25],[24,0,2875],[25,0,3593.75],[26,0,4312.5],[27,0,5031.25]]


R.wait_start()#start command
for i in range(5):
		seen_markers = get_markers()
		circles = []
		for i in seen_markers:
				point = calculate_distance(i[0],i[1],i[2], i[3])
				circles.append([marker_list[i[3]][1],marker_list[i[3]][2],point])
		print(circles)
		valid_points = [[],[]]
		for i in range(len(circles)):
				if i +1 != len(circles):
						a= i+1
				else:
						a = 0
				points = get_intersections(circles[i][0],circles[i][1],circles[i][2], circles[a][0],circles[a][1],circles[a][2])
				for i in points:
						if 0<i[0]<5750 and 0<i[1]<5750:
								valid_points[0].append(i[0])
								valid_points[1].append(i[1])

		avg_x = sum(valid_points[0]) / len(valid_points[0])
		avg_y = sum(valid_points[1]) / len(valid_points[1])
		print(avg_x, avg_y)
		print('Complete\nFound position - coordinate('+str(avg_x)+','+str(avg_y)+')\nContinueing in 5s')
		R.power_board.piezo.buzz(0.2, 400)
		time.sleep(0.2)
		R.power_board.piezo.buzz(0.2, 400)
		time.sleep(5)
print("Done.")
R.power_board.piezo.buzz(0.2, 400)
time.sleep(0.2)
R.power_board.piezo.buzz(0.2, 400)
time.sleep(0.2)
R.power_board.piezo.buzz(0.2, 400)
time.sleep(0.2)
