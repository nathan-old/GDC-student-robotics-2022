'''
medium single beep = retry
long single beep = turn
double beep = got new position
quadruple beep = got all positions
'''
from math import *
from sr.robot3 import *
import os
import time
rpm_data = [[0,-0.1,-0.2,-0.3,-0.4,-0.5,-0.6,-0.7,-0.8,-0.9,-1.0],[0,7.042253521,18.01801802,30.92783505,42.55319149,50.84745763,65.2173913,85.71428571,88.23529412,89.82035928,90.09009009]]

R = Robot(auto_start=True, verbose=True)
wheels = [R.motor_boards['SR0PJ1W'].motors[0],R.motor_boards['SR0PJ1W'].motors[1],R.motor_boards['SR0VG1M'].motors[0]] #array of motor commands
arm_radius = 0.28 # radius of the arms of the robot/ m
circumfrence = 0.3299
motor_power = 0.1
#rpm = rpm_data[1][1]
rpm = 122.03*(motor_power)
speed = circumfrence * (rpm/60) # speed of the motors at power = 0.5/ m/s
print(rpm)
data_filename = os.path.join(R.usbkey, 'data')

def save_data(data, mode):
	output = open(data_filename+'.txt',mode)
	print('OPENED ' + data_filename+'.txt SUCESSFULLY')
	if mode != 'w':
		output.write(data+'\n')
	output.close()
def rotate(angle):
	distance = angle * ((pi * 2 * arm_radius) / 360)
	print('Turning ' + str(angle) + ' degrees')
	time_ = float(distance / speed)
	for wheel in wheels:
		wheel.power = motor_power
	time.sleep(time_)
	for wheel in wheels:
		wheel.power = BRAKE

def reverse(x_distance):
	distance = x_distance/cos(radians(30))
	print('Moving '+str(x_distance)+' meters backwards')
	time_ = float(distance / speed)
	wheels[0].power = motor_power
	wheels[1].power = motor_power*-1
	time.sleep(time_)
	wheels[0].power = BRAKE
	wheels[1].power = BRAKE

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

def calculate_distance(theta,r):
	horizontal_distance = (r*cos(radians(theta)))
	return horizontal_distance

def get_markers():
	'''
	inputs from robot camera in degrees and mm
	list of markers seen by camera, in order theta, psi, r, marker number
	'''
	turn_what_angle = 10
	rotation = 0
	retry_count = 0
	markers = []
	while len(markers) != 2:
		markers = []
		time.sleep(0.5)
		markers_input = R.camera.see()
		for marker in markers_input:
			markers.append([degrees(marker.spherical.rot_x),degrees(marker.spherical.rot_y),marker.spherical.dist,marker.id])
		if len(markers) != 2:
			retry_count += 1
			if retry_count < 3:
				R.power_board.piezo.buzz(0.5, 400)
				print('Not enough markers, trying again')
				markers = []
			else:
				R.power_board.piezo.buzz(1, 400)
				rotate(turn_what_angle)
				rotation += turn_what_angle
				if rotation == 360:
					rotation = 0
					print('reached full turn, moving back')
					reverse(0.2)
	return markers
save_data('','w')
# list of marker positions
marker_list = [[0,718.75,5750],[1,1437.5,5750],[2,2156.25,5750],[3,2875,5750],[4,3593.75,5750],[5,4312.5,5750],[6,5031.25,5750],[7,5750,5031.25],[8,5750,4312.5],[9,5750,3593.75],[10,5750,2875],[11,5750,2156.25],[12,5750,1437.5],[13,5750,718.75],[14,5031.25,0],[15,4312.5,0],[16,3593.75,0],[17,2875,0],[18,2156.25,0],[19,1437.5,0],[20,718.75,0],[21,0,718.75],[22,0,1437.5],[23,0,2156.25],[24,0,2875],[25,0,3593.75],[26,0,4312.5],[27,0,5031.25]]


R.wait_start()#start 
start_time = time.time()
for i in range(5):
		seen_markers = get_markers()
		save_data(str(seen_markers),'a')
		circles = []
		for i in seen_markers:
			radius = calculate_distance(i[0],i[2])
			circles.append([marker_list[i[3]][1],marker_list[i[3]][2],radius])
		#print(circles)
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
		if len(valid_points[0]) > 1:
			avg_x = sum(valid_points[0]) / len(valid_points[0])
			avg_y = sum(valid_points[1]) / len(valid_points[1])
			print('-------------------------------------------------------------------------')
			print(seen_markers)
			print('Found position - coordinate('+str(round(avg_x,3))+','+str(round(avg_y,3))+')-using average of 2\nContinueing in 5s')
			print('-------------------------------------------------------------------------')
			#save_data(str([avg_x,avg_y]),'a')
		elif len(valid_points[0]) == 1:
			avg_x = valid_points[0]
			avg_y = valid_points[1]
			print('-------------------------------------------------------------------------')
			print(seen_markers)
			print('Found position - coordinate('+str(round(avg_x,3))+','+str(round(avg_y,3))+')-using one point\nContinueing in 5s')
			print('-------------------------------------------------------------------------')
		else:
			print('no valid intersections - this should not run')
		R.power_board.piezo.buzz(0.2, 400)
		time.sleep(0.2)
		R.power_board.piezo.buzz(0.2, 400)
		time.sleep(4.4)
end_time = time.time()
print('Data file saved to: ' + data_filename + '.txt')
print('finished in '+str(end_time-start_time)+' Seconds')
R.power_board.piezo.buzz(0.2, 400)
time.sleep(0.2)
R.power_board.piezo.buzz(0.2, 400)
time.sleep(0.2)
R.power_board.piezo.buzz(0.2, 400)
time.sleep(0.2)
R.power_board.piezo.buzz(0.2, 400)
