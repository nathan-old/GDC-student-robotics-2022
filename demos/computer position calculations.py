from math import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def get_intersections(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d=sqrt((x1-x0)**2 + (y1-y0)**2)
    
    # non intersecting
    if d > r0 + r1 :
        return None
    # One circle within other
    if d < abs(r0-r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
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


# calculates the distance of the marker from the robot
def calculate_distance(theta,psi,r, num):
	h = (r*cos(radians(90-theta)))
	z = (r*cos(radians(theta)))
	x = (h*cos(radians(90-psi)))
	y = (h*sin(radians(90-psi)))
	print("x:",x,end = ', ')
	print("y:",y,end = ', ')
	print("z:",z)
	return calculate_pos(x, y, marker_list, num, h)

# calculates the position of the robot using the position of the marker
def calculate_pos(x_in,y_in,marker_list,marker_number, h):
	robot_pos = []
	for i in range(len(marker_list)):
			if marker_list[i][0] == marker_number:
				marker_pos = [marker_list[i][1],marker_list[i][2]]
	if marker_number >= 0 and marker_number <= 6:
		x = marker_pos[0]-y_in
		y = marker_pos[1]-x_in
		robot_pos = [x,y,h]
	elif marker_number >= 7 and marker_number <= 13:
		x = marker_pos[0]-x_in
		y = marker_pos[1]+y_in
		robot_pos = [x,y,h]
	elif marker_number >= 14 and marker_number <= 20:
		x = marker_pos[0]+y_in
		y = marker_pos[1]+x_in
		robot_pos = [x,y,h]
	elif marker_number >= 19 and marker_number <= 27:
		x = marker_pos[0]+x_in
		y = marker_pos[1]+y_in
		robot_pos = [x,y,h]
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
seen_markers = [[90,90,1000,23],[90,90-35.678,1231.1,22], [90,90+35.678,1231.1,24]]

#[90,90,5750/2,24],[90,90,5750/2,17],

#seen_markers.append([-2.415157355926346, -11.116173340480376, 2086000, 0])
marker_list = [[0,718.75,5750],[1,1437.5,5750],[2,2156.25,5750],[3,2875,5750],[4,3593.75,5750],[5,4312.5,5750],[6,5031.25,5750],[7,5750,5031.25],[8,5750,4312.5],[9,5750,3593.75],[10,5750,2875],[11,5750,2156.25],[12,5750,1437.5],[13,5750,718.75],[14,5031.25,0],[15,4312.5,0],[16,3593.75,0],[17,2875,0],[18,2156.25,0],[19,1437.5,0],[20,718.75,0],[21,0,718.75],[22,0,1437.5],[23,0,2156.25],[24,0,2875],[25,0,3593.75],[26,0,4312.5],[27,0,5031.25]]


plt.rcParams["figure.figsize"] = (6,6)
plt.xlim(0,5750)
plt.ylim(0,5750)
plt.gca().set_aspect('equal')
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
#plt.axvspan(0, 2000, alpha=0.7)
#plt.axvspan(3750, 5750, alpha=0.7)
#plt.axhspan(0, 2000, alpha=0.7)
#plt.axhspan(3750, 5750, alpha=0.7)
#plt.gca().add_patch(patches.Rectangle((2000,2275),1750 ,1200, alpha=0.7))
#plt.gca().add_patch(patches.Rectangle((2275,2000),1200, 1750, alpha=0.7))
#plt.gca().add_patch(patches.Rectangle((2275,2275),1200, 1200, alpha=1, color='b'))

for i in range(len(x)):
	plt.text(x[i], y[i], str(i), horizontalalignment='center', verticalalignment='center', fontsize = 5, color='w')

circles = []
for i in seen_markers:
	point = calculate_distance(i[0],i[1],i[2], i[3])
	#plt.scatter(point[0],point[1], color = 'C1')
	circles.append([marker_list[i[3]][1],marker_list[i[3]][2],point[2]])
	#plt.text(point[0],point[1], str(i[3]), horizontalalignment='center', verticalalignment='center', fontsize = 5, color='w')
	plt.gca().add_patch(patches.Circle((marker_list[i[3]][1],marker_list[i[3]][2]),point[2], fill = False, linestyle=':',color = 'C1'))
	#plt.plot([point[0], marker_list[i[3]][1]], [point[1], marker_list[i[3]][2]], linestyle=':',color = 'C1')
valid_points = []
for i in range(len(circles)):
	if i +1 != len(circles):
		a= i+1
	else:
		a = 0
	points = get_intersections(circles[i][0],circles[i][1],circles[i][2], circles[a][0],circles[a][1],circles[a][2])
	for i in points:
		if 0<i[0]<5750 and 0<i[1]<5750:
			valid_points.append(i)
avg_x = 0
avg_y = 0
for i in valid_points:
	avg_x += i[0]
	avg_y += i[1]
avg_x /= len(valid_points)
avg_y /= len(valid_points)
plt.scatter(avg_x,avg_y, color = 'C3')
#print(len(valid_points), valid_points)


plt.savefig(fname = 'figure.png', dpi = 300, transparent = True)