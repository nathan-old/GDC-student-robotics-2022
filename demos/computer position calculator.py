from math import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches

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
		y = marker_pos[1]+y_in
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

# list of marker positions
marker_list = [[0,718.75,5750],[1,1437.5,5750],[2,2156.25,5750],[3,2875,5750],[4,3593.75,5750],[5,4312.5,5750],[6,5031.25,5750],[7,5750,5031.25],[8,5750,4312.5],[9,5750,3593.75],[10,5750,2875],[11,5750,2156.25],[12,5750,1437.5],[13,5750,718.75],[14,5031.25,0],[15,4312.5,0],[16,3593.75,0],[17,2875,0],[18,2156.25,0],[19,1437.5,0],[20,718.75,0],[21,0,718.75],[22,0,1437.5],[23,0,2156.25],[24,0,2875],[25,0,3593.75],[26,0,4312.5],[27,0,5031.25]]
central_box = [[2275,2275],[2275,3475],[3475,3475],[3475,2275],[2275,2275]]
arena_border = [[0,0],[0,5750],[5750,5750],[5750,0],[0,0]]
can_list = [[2875,33.5],[33.5,2875],[2875,5716.5],[5716.5,2875],[900,900],[4850,900],[4850,4850],[900,4850],[1135,2875],[2875,1135],[4615,2875],[2875,4615]]
point_zone = [[2500,0],[3250,0],[5750,2500],[5750,3250],[3250,5750],[2500,5750],[0,3250],[0,2500],[2500,0]]
start_boxes = [[0,1000],[1000,1000],[1000,0],[4750,0],[4750,1000],[5750,1000],[5750,4750],[4740,4750],[4750,5750],[1000,5750],[1000,4750],[0,4750],[0,1000]]



# inputs from robot camera in degrees and mm
# list of markers seen by camera, in order theta, psi, r, marker number
seen_markers = [[85,45,750,11],[90,90,800,8], [90,90,5750/2,24],[90,90,5750/2,17],[90,90,1000,23],[90,90-35.678,1231.1,22], [90,90+35.678,1231.1,24]]



#seen_markers.append([-2.415157355926346, -11.116173340480376, 2086000, 0])
marker_list = [[0,718.75,5750],[1,1437.5,5750],[2,2156.25,5750],[3,2875,5750],[4,3593.75,5750],[5,4312.5,5750],[6,5031.25,5750],[7,5750,5031.25],[8,5750,4312.5],[9,5750,3593.75],[10,5750,2875],[11,5750,2156.25],[12,5750,1437.5],[13,5750,718.75],[14,5031.25,0],[15,4312.5,0],[16,3593.75,0],[17,2875,0],[18,2156.25,0],[19,1437.5,0],[20,718.75,0],[21,0,718.75],[22,0,1437.5],[23,0,2156.25],[24,0,2875],[25,0,3593.75],[26,0,4312.5],[27,0,5031.25]]


plt.rcParams["figure.figsize"] = (6,6)
x = []
y = []
for i in range(len(start_boxes)):
	x.append(start_boxes[i][0])
	y.append(start_boxes[i][1])
plt.plot(x,y,'b')

x = []
y = []
for i in range(len(central_box)):
	x.append(central_box[i][0])
	y.append(central_box[i][1])
plt.plot(x,y)
x = []
y = []
for i in range(len(point_zone)):
	x.append(point_zone[i][0])
	y.append(point_zone[i][1])
plt.plot(x,y, linestyle='--', color = 'r')
x = []
y = []
for i in range(len(arena_border)):
	x.append(arena_border[i][0])
	y.append(arena_border[i][1])
plt.plot(x,y,'b')
x = []
y = []
for i in range(len(marker_list)):
	x.append(marker_list[i][1])
	y.append(marker_list[i][2])
plt.scatter(x,y,color='b')
plt.axvspan(0, 2000, alpha=0.7)
plt.axvspan(3750, 5750, alpha=0.7)
plt.axhspan(0, 2000, alpha=0.7)
plt.axhspan(3750, 5750, alpha=0.7)
plt.gca().add_patch(patches.Rectangle((2000,2275),1750 ,1200, alpha=0.7))
plt.gca().add_patch(patches.Rectangle((2275,2000),1200, 1750, alpha=0.7))
plt.gca().add_patch(patches.Rectangle((2275,2275),1200, 1200, alpha=1, color='b'))

for i in range(len(x)):
	plt.text(x[i], y[i], str(i), horizontalalignment='center', verticalalignment='center', fontsize = 5, color='w')


for i in seen_markers:
	point = calculate_distance(i[0],i[1],i[2], i[3])
	plt.scatter(point[0],point[1], color = 'C1')
	plt.text(point[0],point[1], str(i[3]), horizontalalignment='center', verticalalignment='center', fontsize = 5, color='w')
	
	plt.plot([point[0], marker_list[i[3]][1]], [point[1], marker_list[i[3]][2]], linestyle=':',color = 'C1')

#plt.fill_between(x, 0, 3000, where=y < 80)
#plt.show()
#plt.fill_between(x, 0, 1, where=y < -theta,
#                facecolor='red', alpha=0.5)
plt.savefig(fname = 'figure.png', dpi = 1000, transparent = True)