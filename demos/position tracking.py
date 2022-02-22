'''
medium single beep = retry
long single beep = turn
double beep = got new position
quadruple beep = got all positions
'''
from sr.robot3 import *
import os, time, math, threading, cv2, datetime 
import matplotlib.pyplot as plt
R = Robot(auto_start=True, verbose=True)
data_array=[]
wheels = [R.motor_boards['SR0PJ1W'].motors[0],R.motor_boards['SR0PJ1W'].motors[1],R.motor_boards['SR0VG1M'].motors[0]] #array of motor commands
arm_radius = 0.28 # radius of the arms of the robot/ m
circumfrence = 0.3299 # wheel circumference/ m
motor_power = 0.4
rpm = 4.6778*(motor_power**6) - 88.46*(motor_power**5) - 9.1788*(motor_power**4) + 82.827*(motor_power**3) + 5.6922*(motor_power**2) + 97.405*(motor_power)
speed = circumfrence * (rpm/60) # speed of the motors at power = 0.5/ m/s
fov = 47.5
print('Running with an rpm of ' + str(rpm) + 'rpm and a power of '+str(motor_power))
# list of marker positions
marker_list = [[0,718.75,5750],[1,1437.5,5750],[2,2156.25,5750],[3,2875,5750],[4,3593.75,5750],[5,4312.5,5750],[6,5031.25,5750],[7,5750,5031.25],[8,5750,4312.5],[9,5750,3593.75],[10,5750,2875],[11,5750,2156.25],[12,5750,1437.5],[13,5750,718.75],[14,5031.25,0],[15,4312.5,0],[16,3593.75,0],[17,2875,0],[18,2156.25,0],[19,1437.5,0],[20,718.75,0],[21,0,718.75],[22,0,1437.5],[23,0,2156.25],[24,0,2875],[25,0,3593.75],[26,0,4312.5],[27,0,5031.25]]

move_what_distance = -200
turn_what_angle = 10
def x_and_y(bearing, distance):
    if 0<bearing<90:
        x = math.sin(math.radians(bearing))*distance
        y = math.cos(math.radians(bearing))*distance
    elif 90<bearing<180:
        x = math.cos(math.radians(bearing-90))*distance
        y = math.sin(math.radians(bearing-90))*distance*-1
    elif 180<bearing<270:
        x = math.sin(math.radians(bearing-180))*distance*-1
        y = math.cos(math.radians(bearing-180))*distance*-1
    else:
        x = math.cos(math.radians(bearing-270))*distance*-1
        y = math.sin(math.radians(bearing-270))*distance
    return x,y
def rotate(angle):
	negative = 1
	original = angle
	if angle < 0:
		angle *= -1
		negative = -1
	distance = angle * ((math.pi * 2 * arm_radius) / 360)
	print('Turning ' + str(round(original,3)) + ' degrees')
	time_ = float(distance / speed)
	for wheel in wheels:
		wheel.power = motor_power*negative
	time.sleep(time_)
	for wheel in wheels:
		wheel.power = BRAKE

def forwards(x_distance):
	negative = 1
	original = x_distance
	if x_distance < 0:
		x_distance *= -1
		negative = -1
	distance = (x_distance/1000)/math.cos(math.radians(30))
	print('Moving '+str(original)+' meters backwards')
	time_ = float(distance / speed)
	wheels[0].power = motor_power*negative
	wheels[1].power = motor_power*-1*negative
	time.sleep(time_)
	wheels[0].power = BRAKE
	wheels[1].power = BRAKE

def get_intersections(x0, y0, r0, x1, y1, r1):
	d=math.sqrt((x1-x0)**2 + (y1-y0)**2)
	if d > r0 + r1 :
		return [[-1,-1],[-1,-1]]
	if d < abs(r0-r1):
		return [[-1,-1],[-1,-1]]
	if d == 0 and r0 == r1:
		return [[-1,-1],[-1,-1]]
	else:
		a=(r0**2-r1**2+d**2)/(2*d)
		h=math.sqrt(r0**2-a**2)
		x2=x0+a*(x1-x0)/d   
		y2=y0+a*(y1-y0)/d   
		x3=x2+h*(y1-y0)/d     
		y3=y2-h*(x1-x0)/d 
		x4=x2-h*(y1-y0)/d
		y4=y2+h*(x1-x0)/d
		return ([[x3, y3], [x4, y4]])

def calculate_distance(theta,r):
	horizontal_distance = (r*math.cos(math.radians(theta)))
	return horizontal_distance

def get_markers(index):
	'''
	inputs from robot camera in degrees and mm
	list of markers seen by camera, in order theta, psi, r, marker number
	'''
	rotation = 0
	retry_count = 0
	markers = []
	while len(markers) < 2:
		markers = []
		markers_input = R.camera.see()
		frame = R.camera.capture()
		for marker in markers_input:
			markers.append([math.degrees(marker.spherical.rot_x),math.degrees(marker.spherical.rot_y),marker.spherical.dist,marker.id])
		print('seen: ' + str(len(markers)) + ' markers')
		if len(markers) < 2 or len(markers) > 3:
			types = '{0:0=3d}'.format(rotation)+'--'+str(retry_count)
			x = threading.Thread(target=save_pic, args=(frame,index,types,))
			x.start()
			retry_count += 1
			if retry_count < 3:
				markers = []
			elif retry_count >= 1:
				#R.power_board.piezo.buzz(1, 400)
				retry_count = 0
				rotate(turn_what_angle)
				rotation += turn_what_angle
			if rotation == 360:
				rotation = 0
				print('reached full turn, moving back')
				forwards(move_what_distance)
	types="A"
	x = threading.Thread(target=save_pic, args=(frame,index,types,))
	x.start()
	return markers

def plot(num,pos,angle, markers):
	plt.clf()
	for i in markers:
		plt.scatter(marker_list[i][1],marker_list[i][2], color='C3')
	plt.rcParams["figure.figsize"] = (6, 6)
	plt.xlim(0, 5750)
	plt.ylim(0, 5750)
	plt.imshow(plt.imread(os.path.join(R.usbkey, 'background.png')), extent=[0, 5750, 0, 5750])
	x_dis,y_dis = x_and_y(angle-(fov/2),7000)
	plt.plot([pos[0],pos[0]+x_dis],[pos[1],pos[1]+y_dis], linestyle='--', color = 'C2')
	x_dis,y_dis = x_and_y(angle+(fov/2),7000)
	plt.plot([pos[0],pos[0]+x_dis],[pos[1],pos[1]+y_dis], linestyle='--', color = 'C2')
	plt.scatter(pos[0],pos[1])
	plt.text(2875, 2875,('pos: ('+str(round(pos[0],0))+','+str(round(pos[1],0))+')'), fontsize = 5, bbox = dict(facecolor = 'red', alpha = 0.5),ha='center', va='center')
	x_dis,y_dis = x_and_y(angle,7000)
	plt.plot([pos[0],pos[0]+x_dis],[pos[1],pos[1]+y_dis])
	plt.savefig(fname = os.path.join(R.usbkey,'plts', 'figure' + '{0:0=2d}'.format(num) + '.png'), dpi = 200, transparent = True)

def save_pic(picture,index,typea):
	aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
	parameters = cv2.aruco.DetectorParameters_create()
	corners, ids, _ = cv2.aruco.detectMarkers(picture, aruco_dict, parameters=parameters)
	picture = cv2.aruco.drawDetectedMarkers(picture, corners, ids)
	cv2.imwrite(os.path.join(R.usbkey, 'captures','capture-' + '{0:0=2d}'.format(index)+ '--' + typea +'.png'), picture)


R.wait_start()#start 
start_time = time.time()
for index in range(30):
	seen_markers = get_markers(index)
	marker_ids = [x[3] for x in seen_markers]
	circles = []
	for i in seen_markers:
		radius = calculate_distance(i[0],i[2])
		circles.append([marker_list[i[3]][1],marker_list[i[3]][2],radius])
	valid_points = [[],[]]
	for i in range(len(circles)):
		if i + 1 != len(circles):
			a = i+1
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
		bearing_array = []
		for i in seen_markers:
			bearing = math.atan2(marker_list[i[3]][1]-avg_x,marker_list[i[3]][2]-avg_y)-math.radians(i[1])
			if bearing < math.radians(0):
				bearing+=math.radians(360)
			elif bearing >math.radians(360):
				bearing -= math.radians(360)
			bearing_array.append(bearing)
		x, y = 0, 0
		for i in bearing_array:
			x += math.cos(i)
			y += math.sin(i)
		bearing = math.degrees(math.atan2(y, x))
		data_array.append([index,[avg_x,avg_y],bearing,	 marker_ids])
		print('-------------------------------------------------------------------------')
		print('Viewing markers: ', end = '')
		for i in marker_ids:
			print(i, end = ' ')
		print('\nFound position - coordinate('+str(round(avg_x,3))+','+str(round(avg_y,3))+')-using average of ' + str(len(valid_points[0])) + ' points\nYour bearing is: ' + str(round(bearing,3)) + '\nContinueing')
		print('-------------------------------------------------------------------------')
	else:
		print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
		print('no valid intersections - this should not run - are markers in the correct locations')
		print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
	face_what_bearing = 0
	turn = (bearing-face_what_bearing)*-1
	if turn > 1 or turn < -1:
		rotate(turn)
	else:
		print('within 1 degree')

print('finished in '+str(datetime.timedelta(seconds=(time.time()-start_time)))+' Seconds')
R.power_board.piezo.buzz(3, 400)
start_time = time.time()
print('saving plots...')
for i in data_array:
	print(data_array.index(i))
	plot(i[0],i[1],i[2], i[3])
print('Done!')
print('finished in '+str(datetime.timedelta(seconds=(time.time()-start_time)))+' Seconds')
R.power_board.piezo.buzz(0.2, 400)
time.sleep(0.2)
R.power_board.piezo.buzz(0.2, 400)
time.sleep(0.2)
R.power_board.piezo.buzz(0.2, 400)
time.sleep(0.2)
R.power_board.piezo.buzz(0.2, 400)